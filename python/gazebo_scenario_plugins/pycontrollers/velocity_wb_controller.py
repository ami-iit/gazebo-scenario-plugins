import osqp
import pathlib
from . import qp
import numpy as np
import scipy.sparse
from . import osqp_utils
import idyntree.bindings as idt
from dataclasses import dataclass
from gym_ignition.rbd import idyntree
from scenario import core as scenario_core
from gym_pushrecovery.utils import euler_integrator
from .references import BaseReferences, JointReferences
from typing import Dict, List, Optional, OrderedDict, Tuple


@dataclass
class Parameters:

    frames_of_holonomic_contacts: OrderedDict[str, str]

    # Gain of postural task
    k_p: float

    # Gains of desired momentum PI
    k_l: float
    k_w: float

    # Minimum contact force to create an holonomic contact
    fz_min: float = 10.0

    # Feet dimensions
    xMin: float = -0.07
    xMax: float = 0.12
    yMin: float = -0.045
    yMax: float = 0.05

    # Friction cone approximation (even number >= 3)
    num_vertexes: int = 8

    # Static and torsional friction
    mu_s: float = 1.0 / 3
    mu_tau: float = 1.0 / 3


@dataclass
class QpSolution:

    v_b: np.ndarray
    ds: np.ndarray
    tau: np.ndarray
    forces: List[np.ndarray]

    @staticmethod
    def Build(dofs: int, data: np.ndarray) -> "QpSolution":

        if data.ndim != 1:
            raise ValueError(data)

        v_b = data[:6]
        ds = data[6:6+dofs]
        tau = data[6+dofs:6+dofs+dofs]
        forces = [f for f in data[6+dofs+dofs:].reshape(-1, 6)]

        return QpSolution(v_b=v_b, ds=ds, tau=tau, forces=forces)


class VelocityWBController:

    def __init__(self,
                 urdf_file: str,
                 model: scenario_core.Model,
                 parameters: Parameters,
                 controlled_joints: List[str] = None,
                 gravity: List[float] = (0, 0, -9.80)):

        self.model = model
        self.urdf_file = urdf_file

        if not model.valid():
            raise ValueError("Model not valid")

        if not pathlib.Path(urdf_file).expanduser().is_file():
            raise FileNotFoundError(urdf_file)

        self.controlled_joints = \
            controlled_joints if controlled_joints is not None else model.joint_names()

        if len(gravity) != 3:
            raise ValueError("Wrong size of gravity vector")

        self.gravity = np.array(gravity)

        self._osqp = None
        self._kindyn = None
        self._integrator = None
        self._integrator_ang_mom = None

        self.weight_postural = 1.0
        self.weight_momentum = 1.0

        self.num_forces = 0
        self.parameters = parameters
        self.initial_control_modes: Dict[str, int] = {}

        self.base_references: Optional[BaseReferences] = None
        self.joint_references: Optional[JointReferences] = None

        self.com_position_error_last = None
        self.joint_position_error_last = None

    def initialize(self) -> None:

        # Trigger the allocation of KinDynComputations
        _ = self.kindyn

        # Get the initial control mode of the controlled joints
        for name in self.controlled_joints:
            self.initial_control_modes[name] = self.model.get_joint(name).control_mode()

        # Select the control mode
        # mode = scenario_core.JointControlMode_force
        # mode = scenario_core.JointControlMode_position
        mode = scenario_core.JointControlMode_velocity_direct

        # Change the control mode
        ok_cm = self.model.set_joint_control_mode(mode, self.controlled_joints)

        if not ok_cm:
            raise RuntimeError("Failed to control the joints in torque")

        # TODO: Initialize the quantities to cache between k and k+1
        self.com_position_error_last = np.zeros(3)
        self.joint_position_error_last = np.zeros(self.dofs)

    def step(self) -> None:

        # Get the links associated to holonomic constraints that are in contact
        frames_of_hol_const = self._get_frames_with_active_holonomic_constraints()

        # Store the number of external forces and if they changed
        use_warm_start = True if len(frames_of_hol_const) == self.num_forces else False
        self.num_forces = len(frames_of_hol_const)

        dofs = len(self.controlled_joints)

        m = self.model.total_mass()
        M = self.kindyn.get_mass_matrix()
        h = self.kindyn.get_bias_forces()
        B = np.vstack([np.zeros(shape=(6, dofs)), np.eye(dofs)])
        Lcen_lin, Lcen_ang = self.kindyn.get_centroidal_momentum()
        J_cmm = self.kindyn.get_centroidal_total_momentum_jacobian()
        dJcmm_nu = np.concatenate([self.kindyn.get_com_bias_acc(), (0, 0, 0)])

        # Number of optimized variables: [nu, tau, f1, f2, ...]
        NUM_VARS = (6 + dofs) + (dofs) + (6 * self.num_forces)

        # ===================
        # E1: System Dynamics
        # ===================

        C_dyn = np.hstack([M / self.dt, -B])

        for frame in frames_of_hol_const:

            J_T = self.kindyn.get_frame_jacobian(frame_name=frame).T
            C_dyn = np.hstack([C_dyn, -J_T])

        b_dyn = M @ self.nu / self.dt - h

        # TODO: take only the joints
        C_dyn = C_dyn[6:, :]
        b_dyn = b_dyn[6:]

        # =========================
        # E2: Holonomic Constraints
        # =========================

        C_hol = None
        b_hol = None

        for frame in frames_of_hol_const:

            J = self.kindyn.get_frame_jacobian(frame_name=frame)
            dJ_nu = self.kindyn.get_frame_bias_acc(frame_name=frame)

            # Build constraints of this contact
            C_this_hol = np.hstack([J / self.dt,
                                    np.zeros(shape=(6, dofs + 6 * self.num_forces))])
            b_this_hol = J @ self.nu / self.dt - dJ_nu

            # Stack with the other contacts
            b_hol = b_this_hol if b_hol is None else np.hstack([b_hol, b_this_hol])
            C_hol = C_this_hol if C_hol is None else np.vstack([C_hol, C_this_hol])

        if self.num_forces > 0:
            assert (C_hol @ np.zeros(NUM_VARS) - b_hol) is not None

        # =====================
        # E3: Momentum Dynamics
        # =====================

        C_mom_dyn = np.hstack([-J_cmm / self.dt, np.zeros(shape=(6, dofs))])

        # Get the base position in world coordinates
        W_H_B = self.kindyn.get_world_base_transform()
        W_p_B = W_H_B[0:3, 3]

        # Build the transform between CoM and base link
        # (rotation does not matter for CoM -> W_R_CoM = W_R_B)
        CoM_p_B = W_p_B - self.kindyn.get_com_position()
        CoMW_H_BW = idt.Transform_Identity()
        CoMW_H_BW.setPosition(idt.Position(CoM_p_B[0], CoM_p_B[1], CoM_p_B[2]))

        CoMW_X_BW = CoMW_H_BW.asAdjointTransformWrench().toNumPy()

        for frame in frames_of_hol_const:

            BW_X_CW = self.AB_X_CD(frameA=self.model.base_frame(),
                                   frameB="world",
                                   frameC=frame,
                                   frameD="world",
                                   kin_dyn=self.kindyn)

            # Stack the adjoint matrices
            C_mom_dyn = np.hstack([C_mom_dyn, CoMW_X_BW @ BW_X_CW])

        gravity_wrench = m * np.concatenate([-self.gravity, (0, 0, 0)])
        b_mom_dyn = gravity_wrench + dJcmm_nu - J_cmm @ self.nu / self.dt

        # =========================
        # E4: Momentum Minimization
        # =========================

        A_mom_min = np.hstack([np.zeros(shape=(6, 6 + dofs + dofs))])

        for frame in frames_of_hol_const:

            # Transform between base and contact frame
            B_H_C: idt.Transform = self.kindyn.kindyn.getRelativeTransform(
                self.kindyn.get_floating_base(), frame)

            # Get the base position in world coordinates
            W_H_B = self.kindyn.get_world_base_transform()
            W_p_B = W_H_B[0:3, 3]

            # Build the transform between CoM and base link
            # (rotation does not matter for CoM -> W_R_CoM = W_R_B)
            CoM_p_B = W_p_B - self.kindyn.get_com_position()
            CoM_H_B = idt.Transform_Identity()
            CoM_H_B.setPosition(idt.Position(CoM_p_B[0], CoM_p_B[1], CoM_p_B[2]))

            # Transform between CoM and link in contact
            CoM_H_C: idt.Transform = CoM_H_B * B_H_C

            # Get the adjoint matrix that expresses the contact force in the CoM
            CoM_X_C: idt.Matrix6x6 = CoM_H_C.asAdjointTransformWrench()

            # Stack the adjoint matrices
            A_mom_min = np.hstack([A_mom_min, CoM_X_C.toNumPy()])

        m = self.model.total_mass()
        # gravity_wrench = - m * self.gravity[2] * np.array([0, 0, 1, 0, 0, 0])  TODO
        gravity_wrench = m * np.array([0, 0, 9.801, 0, 0, 0])

        # a_mom_min = -gravity_wrench
        a_mom_min = gravity_wrench

        # ====================
        # E5: Desired Momentum
        # ====================

        # Build the desired momentum from the base velocity references
        nu_des = np.concatenate([self.base_references.linear_velocity,
                                 self.base_references.angular_velocity,
                                 self.joint_references.velocity])
        Lcen_des = J_cmm @ nu_des

        # Calculate the error
        error = np.zeros(6)
        error[0:3] = self.parameters.k_l * m * self.com_position_error_last
        error[3:6] = self.parameters.k_w * self.integrator_ang_mom.step(value=Lcen_ang)

        # Get the constraint data
        b_mom_des = Lcen_des - error
        C_mom_des = np.hstack([J_cmm, np.zeros(shape=(6, dofs + 6 * self.num_forces))])

        # Get the current base position
        W_H_B = self.kindyn.get_world_base_transform()
        base_position = W_H_B[0:3, 3]

        # Get the desired CoM position from the base reference
        com_position_desired = self.base_references.position + \
                               (self.kindyn.get_com_position() - base_position)

        # Update the error for the next step
        # Note: this is equivalent to (p_B - p_des_B)
        self.com_position_error_last = \
            self.kindyn.get_com_position() - com_position_desired

        assert (C_mom_des @ np.zeros(NUM_VARS) - b_mom_des) is not None

        # ============
        # T1: Postural
        # ============

        A_p = np.hstack([np.zeros(shape=(dofs, 6)),
                         np.eye(dofs),
                         np.zeros(shape=(dofs, dofs + 6 * self.num_forces))])

        a_p = self.joint_references.velocity - \
              self.parameters.k_p * self.joint_position_error_last

        self.joint_position_error_last = \
            self.kindyn.get_joint_positions() - self.joint_references.position

        # ==============================
        # T2: Joint Torques Minimization
        # ==============================

        A_tau = np.hstack([np.zeros(shape=(dofs, 6 + dofs)),
                           np.eye(dofs),
                           np.zeros(shape=(dofs, 6 * self.num_forces))])

        a_tau = np.zeros(dofs)

        # ================================
        # T2b: Joint Velocity Minimization
        # ================================

        A_vel = np.hstack([np.zeros(shape=(dofs, 6)),
                           np.eye(dofs),
                           np.zeros(shape=(dofs, dofs)),
                           np.zeros(shape=(dofs, 6 * self.num_forces))])

        a_vel = np.zeros(dofs)

        # =================
        # I1. Friction cone
        # =================

        C_wr = None
        u_wr = None

        for frame in frames_of_hol_const:

            C_this_wr, u_this_wr = self._get_contact_wrench_inequality_constraints(frame)

            u_wr = u_this_wr if u_wr is None else np.hstack([u_wr, u_this_wr])
            C_wr = C_this_wr if C_wr is None else np.vstack([C_wr, C_this_wr])

        # =====================
        # I2. Torque saturation
        # =====================

        C_tau_max = np.hstack([np.zeros(shape=(dofs, 6 + dofs)),
                               np.eye(dofs),
                               np.zeros(shape=(dofs, 6 * self.num_forces))])
        u_tau_max = np.ones(dofs) * 100.0

        # ==============================
        # Build the equality constraints
        # ==============================

        l, u, C = qp.ConstraintsBuilder(num_vars=NUM_VARS). \
            add_equality(C=C_dyn, b=b_dyn). \
            add_equality(C=C_mom_dyn, b=b_mom_dyn). \
            add_bound(C=C_tau_max, u=u_tau_max, l=-u_tau_max). \
            build()

        if self.num_forces > 0:

            l, u, C = qp.ConstraintsBuilder(num_vars=NUM_VARS, l=l, u=u, C=C). \
                add_equality(C=C_hol, b=b_hol). \
                add_upper_bound(C=C_wr, u=u_wr). \
                build()

        # ==================================
        # Build the Hessian and the Gradient
        # ==================================

        self.weight_postural = 10.0
        self.weight_momentum = 1.0

        H, g = qp.CostBuilder(num_vars=NUM_VARS). \
            add_task(A=A_p, a=a_p, weight=self.weight_postural). \
            add_task(A=A_vel, a=a_vel, weight=0.1). \
            add_task(A=A_tau, a=a_tau, weight=0.001). \
            build()

        # The base can be controlled only with contacts
        if self.num_forces > 0:
            H, g = qp.CostBuilder(num_vars=NUM_VARS, H=H, g=g). \
                add_task(A=C_mom_des, a=b_mom_des, weight=self.weight_momentum). \
                build()

        # ============
        # Solve the QP
        # ============

        use_warm_start = False
        # use_warm_start = True
        if self._osqp is None or not use_warm_start:
            self._osqp = None
            self.osqp.setup(P=scipy.sparse.csc_matrix(H),
                            q=g,
                            A=scipy.sparse.csc_matrix(C),
                            l=l, u=u,
                            verbose=True,
                            # verbose=False,
                            # polish=True,
                            warm_start=False,
                            )
        else:
            self._osqp = None
            self.osqp.setup(P=scipy.sparse.csc_matrix(H),
                            q=g,
                            A=scipy.sparse.csc_matrix(C),
                            l=l, u=u,
                            verbose=True,
                            # verbose=False,
                            # polish=True,
                            warm_start=False,
                            )
            self.osqp.warm_start(x=self.result.x, y=self.result.y)
            # self.osqp.update(Px=scipy.sparse.triu(H).data,
            #                  q=g,
            #                  #Ax=scipy.sparse.csc_matrix(C),
            #                  Ax=C,
            #                  l=l, u=u)

        self.result = osqp_utils.OSQPResult.Build(result=self.osqp.solve())

        if self.result.info.status_val != osqp_utils.OSQPStatus.OSQP_SOLVED.value:
            raise RuntimeError("OSQP failed to solve the problem")

        # Build the solution from the result
        solution = QpSolution.Build(dofs=dofs, data=self.result.x)
        print(solution)
        print()

        for frame, force in zip(frames_of_hol_const, solution.forces):
            print(frame, force)

        print()
        print(f"H_cost={self.result.x.T @ H @ self.result.x}")
        print(f"g_cost={self.result.x.T @ g}")

        for name, A, a in [
            ("postural", A_p, a_p),
            ("torques", A_tau, a_tau),
            ("momdes", C_mom_des, b_mom_des),
            # (C_mom_dyn, b_mom_dyn),
            # (A_mom_min, a_mom_min),
            # ("forces", A_force, a_force),
            ("velocities", A_vel, a_vel),
        ]:

            if A is None or a is None:
                continue

            H, g = qp.CostBuilder(num_vars=NUM_VARS). \
                add_task(A=A, a=a). \
                build()
            # print("=")
            # print(results.x.T @ H @ results.x)
            # print(results.x.T @ g)
            print(self.result.x.T @ H @ self.result.x + self.result.x.T @ g, f"\t{name}")
            # print(results.x.T @ H @ results.x + results.x.T @ g + a.T @ a)
        print()

        # ok_ref = self.model.set_joint_generalized_force_targets(
        #     solution.tau.tolist(), self.controlled_joints)
        ok_ref = self.model.set_joint_velocity_targets(
            solution.ds.tolist(), self.controlled_joints)

        if not ok_ref:
            raise RuntimeError("Failed to set the joint torques targets")

    def terminate(self) -> None:

        self._kindyn = None

        # Restore the initial control mode of the controlled joints
        for name, mode in self.initial_control_modes.items():
            self.model.get_joint(name).set_control_mode(mode=mode)

    def update_state_from_model(self) -> None:

        self.kindyn.set_robot_state_from_model(model=self.model,
                                               world_gravity=self.gravity)

    def set_joint_references(self, references: JointReferences) -> None:

        if not references.valid(dofs=self.dofs):
            raise ValueError(references)

        self.joint_references = references

    def set_base_references(self, references: BaseReferences) -> None:

        if not references.valid():
            raise ValueError(references)

        self.base_references = references

    # ==========
    # Properties
    # ==========

    @property
    def dt(self) -> float:

        return self.model.controller_period()

    @property
    def kindyn(self) -> idyntree.kindyncomputations.KinDynComputations:

        if self._kindyn is not None:
            return self._kindyn

        kindyn = idyntree.kindyncomputations.KinDynComputations(
            model_file=self.urdf_file, considered_joints=self.controlled_joints)

        self._kindyn = kindyn
        return self._kindyn

    @property
    def dofs(self) -> int:

        return len(self.controlled_joints)

    @property
    def osqp(self) -> osqp.OSQP:

        if self._osqp is not None:
            return self._osqp

        solver = osqp.OSQP()

        self._osqp = solver
        return self._osqp

    @property
    def nu(self) -> np.ndarray:

        return self.kindyn.get_model_velocity()

    @property
    def integrator(self) -> euler_integrator.EulerIntegrator:

        if self._integrator is not None:
            return self._integrator

        q0 = self.model.joint_positions(self.controlled_joints)

        integrator = euler_integrator.EulerIntegrator(dt=self.dt)
        integrator.set_initial_condition(x0=np.array(q0))

        self._integrator = integrator
        return self._integrator

    @property
    def integrator_ang_mom(self) -> euler_integrator.EulerIntegrator:

        if self._integrator_ang_mom is not None:
            return self._integrator_ang_mom

        integrator = euler_integrator.EulerIntegrator(dt=self.dt)
        integrator.set_initial_condition(x0=np.zeros(3))

        self._integrator_ang_mom = integrator
        return self._integrator_ang_mom

    # =================
    # Private Resources
    # =================

    def _get_frames_with_active_holonomic_constraints(self) -> List[str]:

        frames_with_active_holonomic_constraints = []

        for frame_name, parent_link in self.parameters.frames_of_holonomic_contacts.items():

            # Select only the links that can make holonomic contacts
            if parent_link not in self.model.links_in_contact():
                continue

            # Get the link
            link = self.model.get_link(link_name=parent_link)

            # Loop over the contacts with other bodies (it includes self collisions)
            for contact in link.contacts():
                assert contact.body_a == link.name(scoped=True)

                # Discard self collisions
                if contact.body_b in self.model.link_names(scoped=True):
                    continue

                # We use the desired minimum vertical force as threshold
                # TODO this is not at the CoP
                if np.linalg.norm(link.contact_wrench()[0:3]) < self.parameters.fz_min:
                    continue

                print(f"Detected {frame_name} with norm={np.linalg.norm(link.contact_wrench()[0:3])}")
                frames_with_active_holonomic_constraints.append(frame_name)

        return frames_with_active_holonomic_constraints

    # TODO: from https://www.cs.miami.edu/home/harald/ics/tvcg.pdf
    def _get_contact_wrench_inequality_constraints(self, link_name: str) -> \
            Tuple[np.ndarray, np.ndarray]:

        # Get the 1-based index of the link name inside the QP output
        this_idx = self._get_frames_with_active_holonomic_constraints().index(link_name) + 1

        # Discretize the normalized circle corresponding of a cone level
        angles = np.linspace(start=0,
                             stop=2*np.pi,
                             endpoint=False,
                             num=self.parameters.num_vertexes)

        # Calculate the xy coordinates in the unit circle
        points = np.array([np.cos(angles), np.sin(angles)]).transpose()

        u_cone = None
        C_cone = None

        # Friction cone inequalities
        for point1, point2 in zip(points, np.roll(points, shift=-1, axis=0)):

            # Extract the sine of the angle (y coordinate)
            c1, s1 = point1
            _, s2 = point2

            # Interpolate a line passing through the two points
            a, b = self._line_params(p1=point1, p2=point2)

            if a == 0.0:
                msg = "Edge case found. Maybe you used an even number of vertices?"
                raise RuntimeError(msg)

            # Check whether it's the upper or lower part of the circle
            # (it affects the sign of the inequality)
            if np.sign(s1) >= 0 and np.sign(s2) > 0:
                upper_plane = 1
            elif np.sign(s1) < 0 and np.sign(s2) <= 0:
                upper_plane = -1
            elif np.sign(s1) > 0 and np.sign(s2) <=0:
                upper_plane = int(np.sign(a))
            elif np.sign(s1) < 0 and np.sign(s2) >= 0:
                upper_plane = -int(np.sign(a))
            else:
                raise RuntimeError("This should not happen")

            # -a * f_x + f_y - b * mu_s * f_z <= 0
            new_C = upper_plane * np.array([[-a, 1., -b * self.parameters.mu_s, 0, 0, 0]])
            new_u = upper_plane * np.array([0])

            u_cone = np.concatenate([u_cone, new_u]) if u_cone is not None else new_u
            C_cone = np.concatenate([C_cone, new_C]) if C_cone is not None else new_C

        # Positive f_z
        C_z = np.array([[0, 0, -1, 0, 0, 0]])  # -f_z <= -fz_min
        u_z = np.array([-self.parameters.fz_min])

        # Torsional friction
        C_tau = np.array([
            [0, 0, -self.parameters.mu_tau, 0, 0,  1],  # -mu * f_z + tau_z <= 0
            [0, 0, -self.parameters.mu_tau, 0, 0, -1],  # -mu * f_z - tau_z <= 0
        ])
        u_tau = np.array([0, 0])

        # CoP inside foot polygon
        C_cop = np.array([
            [0, 0,  self.parameters.xMin, 0,  1,  0],  # +xMin * f_z + tau_y <= 0
            [0, 0, -self.parameters.xMax, 0, -1,  0],  # -xMax * f_z - tau_y <= 0
            [0, 0, -self.parameters.yMax,  1, 0,  0],  # -yMax * f_z + tau_x <= 0
            [0, 0,  self.parameters.yMin, -1, 0,  0],  # +yMin * f_z - tau_x <= 0
        ])
        u_cop = np.array([0, 0, 0, 0])

        # Stack all the values
        C = np.concatenate([
            C_cone,
            C_z,
            C_tau,
            C_cop,
        ])
        u = np.concatenate([
            u_cone,
            u_z,
            u_tau,
            u_cop,
        ])

        # Extend C to the full QP output adding zeros for the nu and other forces
        # dofs = len(self.controlled_joints)
        nu_zeros = np.zeros(shape=[C.shape[0], self.dofs + 6])
        tau_zeros = np.zeros(shape=[C.shape[0], self.dofs])

        num_wrenches_pre = this_idx - 1
        num_wrenches_post = self.num_forces - num_wrenches_pre - 1
        wrenches_pre = np.zeros(shape=[C.shape[0], 6 * num_wrenches_pre])
        wrenches_post = np.zeros(shape=[C.shape[0], 6 * num_wrenches_post])

        C = np.hstack([nu_zeros, tau_zeros, wrenches_pre, C, wrenches_post])

        return C, u

    @staticmethod
    def _line_params(p1: np.ndarray, p2: np.ndarray) \
        -> Tuple[float, float]:
        """
        Return the a and b coefficients of the line ``y = ax + b`` passing through
        the input points p1 and p2.
        """

        if p1.size != 2 or p2.size != 2:
            raise ValueError

        a = (p1[1] - p2[1]) / (p1[0] - p2[0])
        b = p1[1] - p1[0] * a

        return a, b

    @staticmethod
    def AB_X_CD(frameA: str,
                frameC: str,
                kin_dyn: idyntree.kindyncomputations,
                frameB: str = None,
                frameD: str = None) -> np.ndarray:

        transform = kin_dyn.get_relative_transform_explicit(
            ref_frame_origin=frameA,
            ref_frame_orientation=frameB,
            frame_origin=frameC,
            frame_orientation=frameD)

        AB_H_CD = idyntree.numpy.FromNumPy.to_idyntree_transform(
            position=transform[0:3, 3], rotation=transform[0:3, 0:3])

        return AB_H_CD.asAdjointTransformWrench().toNumPy()
