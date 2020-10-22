import ray.rllib
import numpy as np
import idyntree.bindings as idt
from collections import OrderedDict
from gym_pushrecovery import models
from scenario import core as scenario_core
from scenario import gazebo as scenario_gazebo
from gym_ignition.utils.scenario import init_gazebo_sim
from gazebo_scenario_plugins.pycontrollers import references
from gazebo_scenario_plugins.pycontrollers import velocity_wb_controller

scenario_gazebo.set_verbosity(scenario_gazebo.Verbosity_debug)

# Get the simulator and the world
gazebo, world = init_gazebo_sim(step_size=0.001, steps_per_run=10)

assert world.set_gravity(gravity=(0, 0, -10.0))

# Show the GUI
import time
gazebo.gui()
gazebo.run(paused=True)
time.sleep(3)

# Insert the iCub model
icub = models.icub.ICubGazeboSimpleCollisions(world=world)
assert icub.enable_self_collisions(enable=False)
[j.to_gazebo().set_coulomb_friction(0.000) for j in icub.joints()]
[j.to_gazebo().set_viscous_friction(0.000) for j in icub.joints()]
[j.to_gazebo().set_max_generalized_force(50.0) for j in icub.joints()]
gazebo.run(paused=True)
# time.sleep(5)

# Set the controller period
icub.set_controller_period(gazebo.step_size() * gazebo.steps_per_run())

# Control all the joints in position. Then the controller will change the mode of the
# controlled joints.
assert icub.set_joint_control_mode(scenario_core.JointControlMode_velocity_direct)

# Controller parameters
parameters = velocity_wb_controller.Parameters(
    frames_of_holonomic_contacts=OrderedDict(l_sole="l_foot", r_sole="r_foot"),
    k_p=2,
    k_l=0.1, k_w=3,
    fz_min=10.0,
    num_vertexes=8,
)

# Select the controlled joints
all_joints = icub.joint_names()
controlled_joints = list(all_joints)
# [controlled_joints.remove(j) for j in all_joints if "wrist" in j]
# [controlled_joints.remove(j) for j in all_joints if "neck" in j]

# Create the controller
controller = velocity_wb_controller.VelocityWBController(
    urdf_file=models.icub.ICubGazeboSimpleCollisions.get_model_file(),
    model=icub,
    parameters=parameters,
    controlled_joints=controlled_joints,
    gravity=world.gravity(),
)

f = 1.0
A = 0.1
T = 15.0

# Y axis (lateral) base trajectory
base_position_0 = icub.base_position()
base_y_position = (A * np.sin(2 * np.pi * f * t)
                   for t in np.arange(start=0.0,
                                      stop=T,
                                      step=gazebo.step_size() * gazebo.steps_per_run()))
base_z_position = (A * np.cos(2 * np.pi * f * t)
                   for t in np.arange(start=0.0,
                                      stop=T,
                                      step=gazebo.step_size() * gazebo.steps_per_run()))

# Initialize the base references
base_references = references.BaseReferences.zero()
base_references.position = np.array(icub.base_position())
base_references.orientation = np.array(icub.base_orientation())

# Initialize the joint references
joint_references = references.JointReferences.zero(len(controlled_joints))
joint_references.position = np.array(icub.joint_positions(controlled_joints))

while not icub.links_in_contact():
    gazebo.run()
gazebo.run()

print()
print("=============")
print("Current state")
print("=============")
print()
print("Contacts:", icub.links_in_contact())
print("Wrench l_foot:", np.array(icub.get_link(link_name="l_foot").contact_wrench()))
print("Wrench r_foot:", np.array(icub.get_link(link_name="r_foot").contact_wrench()))
print()
print("Base position:", np.array(icub.base_position()))
print("Base orientation:", np.array(icub.base_orientation()))
print()
print("Joint positions:", np.array(icub.joint_positions(controlled_joints)))
print("Joint velocities:", np.array(icub.joint_velocities(controlled_joints)))
print()

# Initialize the controller
controller.initialize()
gazebo.run(paused=True)

i = 0
for i, t in enumerate(np.arange(start=0.0,
                                stop=T,
                                step=gazebo.step_size() * gazebo.steps_per_run())):

    if i == 50:
        assert icub.get_link("chest").apply_world_force((100., 0, 0), duration=0.05)

    print()
    print(">>>=======")
    print("References")
    print(">>>=======")
    print()
    print(base_references)
    print(joint_references)
    print()
    print("Contacts:", icub.links_in_contact())
    print("Wrench l_foot:", np.array(icub.get_link(link_name="l_foot").contact_wrench()))
    print("Wrench r_foot:", np.array(icub.get_link(link_name="r_foot").contact_wrench()))
    print()

    # Set joints references
    controller.set_joint_references(references=joint_references)

    # Set base references
    # base_references.position = base_position_0 + np.array([0.0, next(base_y_position), 0.0])
    # base_references.position = base_position_0 + np.array([0.0, 0.0, next(base_z_position) - 1.0])
    controller.set_base_references(references=base_references)

    # Update the state
    controller.update_state_from_model()

    # Step the controller
    controller.step()

    # Run the simulator
    gazebo.run()

    # =================================
    # Express the wrenches at the soles
    # =================================

    controller.update_state_from_model()
    kindyn = controller.kindyn.kindyn

    w_lfoot_wrench: idt.Wrench = idt.Wrench_FromPython(icub.get_link(link_name="l_foot").contact_wrench())
    w_rfoot_wrench: idt.Wrench = idt.Wrench_FromPython(icub.get_link(link_name="r_foot").contact_wrench())

    LSoleW_X_LFootW = controller.AB_X_CD(frameA="l_sole", frameB="world",
                                         frameC="l_foot", frameD="world",
                                         kin_dyn=controller.kindyn)
    RSoleW_X_RFootW = controller.AB_X_CD(frameA="r_sole", frameB="world",
                                         frameC="r_foot", frameD="world",
                                         kin_dyn=controller.kindyn)

    solution = velocity_wb_controller.QpSolution.Build(dofs=controller.dofs,
                                                       data=controller.result.x)
    print(f"ds_meas={np.array(icub.joint_velocities(controlled_joints))}")
    print(f"ds_opti={solution.ds}")
    print()
    print(f"tau_meas={np.array(icub.joint_generalized_forces(controlled_joints))}")
    print(f"tau_opti={solution.tau}")
    print()
    print("Wrench l_sole:", LSoleW_X_LFootW @ np.array(icub.get_link(link_name="l_foot").contact_wrench()))
    print("Wrench r_sole:", RSoleW_X_RFootW @ np.array(icub.get_link(link_name="r_foot").contact_wrench()))
    print()
    print(icub.links_in_contact())

controller.terminate()
gazebo.close()
