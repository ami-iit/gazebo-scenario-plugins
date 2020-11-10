import pickle
import ray.rllib
import numpy as np
from pathlib import Path
# import gym_ignition_models
from gym_ignition import rbd
# import idyntree.bindings as idt
# from dataclasses import dataclass
from gym_pushrecovery import models
from collections import OrderedDict
import bipedal_locomotion.bindings as blf
from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo
from gym_ignition.rbd.conversions import Quaternion
# from gym_ignition.utils import inverse_kinematics_nlp
from gym_ignition.utils.scenario import init_gazebo_sim
from gym_ignition.rbd.idyntree import kindyncomputations
from gym_ignition.rbd.idyntree import inverse_kinematics_nlp
from gazebo_scenario_plugins.pycontrollers import references
from gazebo_scenario_plugins.pycontrollers import velocity_wb_controller

scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)


def create_contact_phase_list(kindyn: kindyncomputations.KinDynComputations) \
        -> blf.ContactPhaseList:

    # Create the map of contact lists
    contact_list_map = dict()

    # Names of the frames
    rfoot_frame = "r_sole"
    lfoot_frame = "l_sole"

    # Create the contact lists
    contact_list_map[rfoot_frame] = blf.ContactList()
    contact_list_map[lfoot_frame] = blf.ContactList()

    # t:   ||    1    2    3    4    5    6    7    8    9    ||
    #      ||----|----|----|----|----|----|----|----|----|----||
    # L:   ||****|    |****|****|****|****|****|         |****||
    # R:   ||****|****|****|              |****|****|****|****||

    # W_H_rfoot = kindyn.get_world_transform(frame_name=rfoot_frame)
    # W_H_lfoot = kindyn.get_world_transform(frame_name=lfoot_frame)

    # p_lfoot, q_lfoot = \
    #     rbd.conversions.Transform.to_position_and_quaternion(transform=W_H_rfoot)
    #
    # p_rfoot, q_rfoot = \
    #     rbd.conversions.Transform.to_position_and_quaternion(transform=W_H_lfoot)
    #
    # print(p_rfoot)
    # raise

    quat =[0., 0, 0, 1]

    # Initial position
    # TODO: if z0 != 0 the solver fails
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([-0.0072, 0.0786, 0.0] + quat),
        activation_time=0.0,
        deactivation_time=1.0)
    assert contact_list_map[rfoot_frame].add_contact(
        new_transform=np.array([-0.0072, -0.0786, 0.0] + quat),
        activation_time=0.0,
        deactivation_time=3.0)

    # Walking
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([0.2, 0.10, 0.0] + quat),
        activation_time=2.0,
        deactivation_time=7.0)
    assert contact_list_map[rfoot_frame].add_contact(
        new_transform=np.array([0.4, -0.10, 0.0] + quat),
        activation_time=6.0,
        deactivation_time=10.0)
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([0.6, 0.10, 0.0] + quat),
        activation_time=9.0,
        deactivation_time=13.0)
    assert contact_list_map[rfoot_frame].add_contact(
        new_transform=np.array([0.8, -0.10, 0.0] + quat),
        activation_time=12.0,
        deactivation_time=16.0)
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([0.8, 0.10, 0.0] + quat),
        activation_time=15.0,
        deactivation_time=16.0)

    # 0. Initial position
    # ===================

    # W_H_rfoot = kindyn.get_world_transform(frame_name=rfoot_frame)
    # W_H_lfoot = kindyn.get_world_transform(frame_name=lfoot_frame)
    #
    # p_lfoot, q_lfoot = \
    #     rbd.conversions.Transform.to_position_and_quaternion(transform=W_H_rfoot)
    #
    # p_rfoot, q_rfoot = \
    #     rbd.conversions.Transform.to_position_and_quaternion(transform=W_H_lfoot)

    # assert contact_list_map[lfoot_frame].add_contact(
    #     new_transform=np.concatenate([p_lfoot, Quaternion.to_xyzw(q_lfoot)]),
    #     activation_time=0.0,
    #     deactivation_time=1.0)
    #
    # assert contact_list_map[rfoot_frame].add_contact(
    #     new_transform=np.concatenate([p_rfoot, Quaternion.to_xyzw(q_rfoot)]),
    #     activation_time=0.0,
    #     deactivation_time=3.0)

    # 1. First step
    # =============

    # p_lfoot += np.array([0.2, 0, 0.2])
    #
    # assert contact_list_map[lfoot_frame].add_contact(
    #     new_transform=np.concatenate([p_lfoot, Quaternion.to_xyzw(q_lfoot)]),
    #     activation_time=2.0,
    #     deactivation_time=6.0)

    # 2. Second step
    # ==============

    # p_rfoot += np.array([0.3, 0, 0])
    #
    # assert contact_list_map[rfoot_frame].add_contact(
    #     new_transform=np.concatenate([p_rfoot, Quaternion.to_xyzw(q_rfoot)]),
    #     activation_time=5.0,
    #     deactivation_time=10.0)

    # 3. Final position
    # =================

    # p_lfoot += np.array([0.1, 0, 0])
    #
    # assert contact_list_map[lfoot_frame].add_contact(
    #     new_transform=np.concatenate([p_lfoot, Quaternion.to_xyzw(q_lfoot)]),
    #     activation_time=9.0,
    #     deactivation_time=10.0)

    # Create the contact phase list
    # =============================

    phase_list = blf.ContactPhaseList()
    phase_list.set_lists(contact_lists=contact_list_map)

    return phase_list

def create_contact_phase_list_in_place(kindyn: kindyncomputations.KinDynComputations) \
        -> blf.ContactPhaseList:

    # Create the map of contact lists
    contact_list_map = dict()

    # Names of the frames
    rfoot_frame = "r_sole"
    lfoot_frame = "l_sole"

    # Create the contact lists
    contact_list_map[rfoot_frame] = blf.ContactList()
    contact_list_map[lfoot_frame] = blf.ContactList()

    quat = [0., 0, 0, 1]

    # Initial position
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([-0.0072, 0.0786, 0.0] + quat),
        activation_time=0.0,
        deactivation_time=1.0)
    assert contact_list_map[rfoot_frame].add_contact(
        new_transform=np.array([-0.0072, -0.0786, 0.0] + quat),
        activation_time=0.0,
        deactivation_time=3.0)

    # Stepping in place
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([0, 0.10, 0.0105] + quat),
        activation_time=2.0,
        deactivation_time=5.0)
    assert contact_list_map[rfoot_frame].add_contact(
        new_transform=np.array([0, -0.10, 0.0105] + quat),
        activation_time=4.0,
        deactivation_time=7.0)
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([0, 0.10, 0.0105] + quat),
        activation_time=6.0,
        deactivation_time=9.0)
    assert contact_list_map[rfoot_frame].add_contact(
        new_transform=np.array([0, -0.10, 0.0105] + quat),
        activation_time=8.0,
        deactivation_time=11.0)
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([0, 0.10, 0.0105] + quat),
        activation_time=10.0,
        deactivation_time=13.0)
    assert contact_list_map[rfoot_frame].add_contact(
        new_transform=np.array([0, -0.0786, 0.0] + quat),
        activation_time=12.0,
        deactivation_time=15.0)
    assert contact_list_map[lfoot_frame].add_contact(
        new_transform=np.array([0, 0.0786, 0.0] + quat),
        activation_time=14.0,
        deactivation_time=15.0)

    # Create the contact phase list
    # =============================

    phase_list = blf.ContactPhaseList()
    phase_list.set_lists(contact_lists=contact_list_map)

    return phase_list


def get_planner_parameters(dt: float) -> blf.StdParametersHandler:

    # Set the parameters
    handler = blf.StdParametersHandler()
    handler.set_parameter_float(name="planner_sampling_time", value=dt)
    handler.set_parameter_int(name="number_of_foot_corners", value=4)

    # Set the foot corners
    handler.set_parameter_vector_float(name="foot_corner_0", value=[0.1, 0.05, 0.0])
    handler.set_parameter_vector_float(name="foot_corner_1", value=[0.1, -0.05, 0.0])
    handler.set_parameter_vector_float(name="foot_corner_2", value=[-0.1, -0.05, 0.0])
    handler.set_parameter_vector_float(name="foot_corner_3", value=[-0.1, 0.05, 0.0])

    # Set the weight of the cost function
    handler.set_parameter_float(name="omega_dot_weight", value=1.0)
    handler.set_parameter_float(name="dcm_tracking_weight", value=1.0)
    handler.set_parameter_float(name="omega_dot_rate_of_change_weight", value=10.0)
    handler.set_parameter_float(name="vrp_rate_of_change_weight", value=100.0)
    handler.set_parameter_float(name="dcm_rate_of_change_weight", value=1.0)

    return handler


def get_initial_state(kindyn: kindyncomputations.KinDynComputations) -> blf.DCMPlannerState:

    # Set the initial state
    initial_state = blf.DCMPlannerState()
    initial_state.dcm_position = kindyn.get_com_position()
    initial_state.dcm_velocity = np.zeros(3)
    initial_state.vrp_position = initial_state.dcm_position
    initial_state.omega = np.sqrt(9.81 / initial_state.dcm_position[2])

    return initial_state


def get_swing_foot_planner(contact_list: blf.ContactList, dt: float) -> blf.SwingFootPlanner:

    parameters_handler = blf.StdParametersHandler()
    parameters_handler.set_parameter_float("sampling_time", dt)
    parameters_handler.set_parameter_float("step_height", 0.1)
    parameters_handler.set_parameter_float("foot_apex_time", 0.5)
    parameters_handler.set_parameter_float("foot_landing_velocity", 0.0)
    parameters_handler.set_parameter_float("foot_landing_acceleration", 0.0)

    planner = blf.SwingFootPlanner()
    assert planner.initialize(handler=parameters_handler)
    planner.set_contact_list(contact_list=contact_list)

    assert planner.is_valid()
    return planner


class DCMtoCoM:

    def __init__(self, com_initial_position: np.ndarray):

        self.com_position = com_initial_position

    def advance(self, state: blf.DCMPlannerState, dt: float) -> None:

        self.com_position += dt * state.omega * (state.dcm_position - self.com_position)

    def get_com_position(self) -> np.ndarray:

        return self.com_position


# =========
# SIMULATOR
# =========

# Get the simulator and the world
gazebo, world = init_gazebo_sim(step_size=0.001, steps_per_run=10)
assert world.set_gravity(gravity=(0, 0, -10.0))
dt = gazebo.step_size() * gazebo.steps_per_run()

# Show the GUI
import time
gazebo.gui()
gazebo.run(paused=True)
time.sleep(3)

# Insert the iCub model
icub = models.icub.ICubGazeboSimpleCollisions(world=world)
assert icub.enable_self_collisions(enable=False)
[j.to_gazebo().set_coulomb_friction(0.001) for j in icub.joints()]
[j.to_gazebo().set_viscous_friction(0.001) for j in icub.joints()]
[j.to_gazebo().set_max_generalized_force(500.0) for j in icub.joints()]
gazebo.run(paused=True)

# Control all the joints in velocity follower (it will not move, no PID tuning needed)
assert icub.set_joint_control_mode(scenario_core.JointControlMode_velocity_follower_dart)

# There is a small transient in the beginning of the simulation
while not icub.links_in_contact():
    gazebo.run()
gazebo.run()

# ==================
# CONFIGURE PLANNERS
# ==================

# Create a KinDynComputations object
kindyn = kindyncomputations.KinDynComputations(
    model_file=icub.get_model_file(), considered_joints=icub.joint_names())
kindyn.set_robot_state_from_model(model=icub, world_gravity=np.array(world.gravity()))

# Create the list of contact phases containing the footsteps
# contact_phase_list = create_contact_phase_list(kindyn=kindyn)
contact_phase_list = create_contact_phase_list_in_place(kindyn=kindyn)

# Get the planner parameters
planner_parameters = get_planner_parameters(dt=dt)

# Initialize the planner
dcm_planner = blf.TimeVaryingDCMPlanner()
assert dcm_planner.initialize(handler=planner_parameters)

# Set the contact configuration
assert dcm_planner.set_contact_phase_list(contact_phase_list=contact_phase_list)

# Set the initial state
dcm_planner.set_initial_state(state=get_initial_state(kindyn=kindyn))

# Initialize the class to convert DCM to CoM
converter = DCMtoCoM(com_initial_position=kindyn.get_com_position())

# Get the swing foot planners
right_swing_planner = get_swing_foot_planner(
    contact_list=contact_phase_list.lists()["r_sole"], dt=dt)
left_swing_planner = get_swing_foot_planner(
    contact_list=contact_phase_list.lists()["l_sole"], dt=dt)

# ============
# CONFIGURE IK
# ============

ik_joints = list(icub.joint_names())
[ik_joints.remove(j) for j in ik_joints if "neck" in j]
[ik_joints.remove(j) for j in ik_joints if "wrist" in j]

ik = inverse_kinematics_nlp.InverseKinematicsNLP(
    urdf_filename=icub.get_model_file(),
    considered_joints=ik_joints,
    joint_serialization=icub.joint_names())

ik.initialize(floating_base=True,
              verbosity=2,
              cost_tolerance=0.001)

# Enable the cartesian targets of the feet
ik.add_target(frame_name="r_sole",
              target_type=inverse_kinematics_nlp.TargetType.POSE,
              as_constraint=True)
ik.add_target(frame_name="l_sole",
              target_type=inverse_kinematics_nlp.TargetType.POSE,
              as_constraint=True)

ik.add_target(frame_name="root_link",
              weight=10.0,
              target_type=inverse_kinematics_nlp.TargetType.ROTATION)
ik.update_rotation_target(
        target_name="root_link",
        quaternion=np.array(icub.base_orientation()))

for link_name, weight in \
    {("chest", 10.0),
     ("head", 1.0),
     ("r_forearm", 1.0),
     ("l_forearm", 1.0),
     ("r_upper_arm", 1.0),
     ("l_upper_arm", 1.0),
     ("r_hand", 1.0),
     ("l_hand", 1.0),
     }:

    ik.add_target(frame_name=link_name,
                  weight=weight,
                  target_type=inverse_kinematics_nlp.TargetType.ROTATION)
    ik.update_rotation_target(
        target_name=link_name,
        quaternion=np.array(icub.get_link(link_name).orientation()))

# Enable the cartesian target of the CoM
ik.add_com_target(as_constraint=True, constraint_tolerance=0.02)

# Set the initial robot configuration
ik.set_current_robot_configuration(
    base_position=np.array(icub.base_position()),
    base_quaternion=np.array(icub.base_orientation()),
    joint_configuration=np.array(icub.joint_positions()))

# ====================
# COMPUTE TRAJECTORIES
# ====================

# use_existing_ik_solution = True
use_existing_ik_solution = False
pickle_filename = "/tmp/ik/step_in_place.pickle"

if use_existing_ik_solution and Path(pickle_filename).exists():

    with open(file=pickle_filename, mode='rb') as file:

        ik_solutions = pickle.load(file=file, fix_imports=False)

else:
    ik_solutions = []

# Compute IK only if there is no precomputed solution
if not ik_solutions:

    # Compute the dcm trajectory
    assert dcm_planner.compute_trajectory()

    for idx in np.arange(start=0, stop=16.0, step=dt):

        # Advance the planners
        dcm_planner.advance()
        left_swing_planner.advance()
        right_swing_planner.advance()

        # Get the planners states
        dcm_planner_state = dcm_planner.get()
        left_foot_state = left_swing_planner.get()
        right_foot_state = right_swing_planner.get()

        # Compute the CoM position from the DCM position
        converter.advance(state=dcm_planner_state, dt=dt)

        # Update the CoM target
        ik.update_com_target(position=converter.get_com_position())

        # Update the right foot target
        ik.update_transform_target(
            target_name="r_sole",
            position=right_foot_state.transform[0:3],
            quaternion=Quaternion.to_wxyz(xyzw=right_foot_state.transform[3:]))

        # Update the left foot target
        ik.update_transform_target(
            target_name="l_sole",
            position=left_foot_state.transform[0:3],
            quaternion=Quaternion.to_wxyz(xyzw=left_foot_state.transform[3:]))

        ik.solve()
        ik_solution = ik.get_full_solution()
        ik_solutions.append(ik_solution)

# =====================
# STORE THE IK SOLUTION
# =====================

if not use_existing_ik_solution and not Path(pickle_filename).is_file():

    with open(file=pickle_filename, mode='wb') as file:

        pickle.dump(obj=ik_solutions,
                    file=file,
                    protocol=pickle.HIGHEST_PROTOCOL,
                    fix_imports=False)

# ===============
# PLAY TRAJECTORY
# ===============

while True:

    for ik_solution in ik_solutions:

        icub.to_gazebo().reset_base_pose(ik_solution.base_position.tolist(),
                                         ik_solution.base_quaternion.tolist())
        icub.to_gazebo().reset_joint_positions(ik_solution.joint_configuration.tolist(),
                                               icub.joint_names())
        gazebo.run(paused=True)
        time.sleep(dt)

# ===================
# VELOCITY CONTROLLER
# ===================

# Controller parameters
parameters = velocity_wb_controller.Parameters(
    frames_of_holonomic_contacts=OrderedDict(l_sole="l_foot", r_sole="r_foot"),
    k_p=5,
    k_l=2, k_w=0,
    fz_min=5.0,
    num_vertexes=6,
)

# Select the controlled joints
all_joints = icub.joint_names()
controlled_joints = list(all_joints)

# Create the controller
controller = velocity_wb_controller.VelocityWBController(
    urdf_file=models.icub.ICubGazeboSimpleCollisions.get_model_file(),
    model=icub,
    parameters=parameters,
    controlled_joints=controlled_joints,
    gravity=world.gravity(),
)

# Initialize the controller
controller.initialize()

base_references = references.BaseReferences.zero()
base_references.position = np.array(icub.base_position())

joint_references = references.JointReferences.zero(len(controlled_joints))
joint_references.position = np.array(icub.joint_positions(controlled_joints))

# ==================
# RUN THE CONTROLLER
# ==================

for ik_solution in ik_solutions:

    base_velocity = (ik_solution.base_position - base_references.position) / dt
    joint_velocity = (ik_solution.joint_configuration - joint_references.position) / dt

    # Set base references
    base_references.position = ik_solution.base_position
    base_references.linear_velocity = base_velocity
    base_references.orientation = ik_solution.base_quaternion
    controller.set_base_references(references=base_references)

    # Set joint references
    joint_references.position = ik_solution.joint_configuration
    joint_references.velocity = joint_velocity
    controller.set_joint_references(references=joint_references)

    print(base_references, icub.base_position(), icub.base_orientation())
    print(joint_references)
    print(icub.joint_positions(controlled_joints))

    # Update the state
    controller.update_state_from_model()

    # Step the controller
    controller.step()

    # Run the simulator
    gazebo.run()
