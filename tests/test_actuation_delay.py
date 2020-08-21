import gym
import pytest
import numpy as np
from queue import Queue
import gym_ignition_models
from gym_ignition import utils
from scenario import core as scenario_core
from gazebo_scenario_plugins import plugins
from gym_ignition.utils.scenario import init_gazebo_sim

# Set the verbosity
utils.logger.set_level(gym.logger.DEBUG)

# Panda PID gains
# https://github.com/mkrizmancic/franka_gazebo/blob/master/config/default.yaml
panda_pid_gains_1000Hz = {
    'panda_joint1': scenario_core.PID(50,    0,  20),
    'panda_joint2': scenario_core.PID(10000, 0, 500),
    'panda_joint3': scenario_core.PID(100,   0,  10),
    'panda_joint4': scenario_core.PID(1000,  0,  50),
    'panda_joint5': scenario_core.PID(100,   0,  10),
    'panda_joint6': scenario_core.PID(100,   0,  10),
    'panda_joint7': scenario_core.PID(10,    0.5, 0.1),
    'panda_finger_joint1': scenario_core.PID(100, 0, 50),
    'panda_finger_joint2': scenario_core.PID(100, 0, 50),
}


def test_actuation_delay():

    # Get the simulator and the world
    gazebo, world = init_gazebo_sim(step_size=0.001,
                                    real_time_factor=1.0,
                                    steps_per_run=1)

    # Get the panda urdf
    panda_urdf = gym_ignition_models.get_model_file("panda")

    # Insert the panda arm
    model_name = "panda"
    assert world.insert_model(panda_urdf,
                              scenario_core.Pose_identity(),
                              model_name)

    # Get the model
    panda = world.get_model(model_name)

    assert panda.valid()
    assert gazebo.run(paused=True)

    # Then, insert the actuation delay plugin.
    # IMPORTANT: it has to be added BEFORE switching to position control mode!
    actuation_delay = plugins.ActuationDelay(delay=50)
    assert panda.to_gazebo().insert_model_plugin(*actuation_delay.args())

    # Set the controller period equal to the physics step (1000Hz)
    panda.set_controller_period(gazebo.step_size())

    # Check that we have gains for all joints
    assert set(panda.joint_names()) == set(panda_pid_gains_1000Hz.keys())

    # Set the PID gains
    for joint_name, pid in panda_pid_gains_1000Hz.items():
        assert panda.get_joint(joint_name).set_pid(pid=pid)

    # Reset joint1 to its middle position
    joint1 = panda.get_joint("panda_joint1").to_gazebo()
    joint1_range = np.abs(joint1.position_limit().max - joint1.position_limit().min)
    joint1_middle = joint1.position_limit().min + joint1_range / 2
    assert joint1.reset_position(joint1_middle)

    # Control the model in position. It sets the current position as new target.
    assert panda.set_joint_control_mode(scenario_core.JointControlMode_position)

    # Get the current positions
    q0 = panda.joint_positions()

    # Build a new reference for a single joint
    q0_j1 = joint1.position()
    q_j1 = (0.9 * joint1_range / 2 * np.sin(2 * np.pi * 0.33 * t)
            for t in np.arange(start=0, stop=10.0, step=gazebo.step_size()))

    # Create a queue
    queue = Queue(maxsize=actuation_delay.delay)

    for _ in range(actuation_delay.delay):

        # Start applying the new target
        j1_ref = q0_j1 + next(q_j1)
        assert joint1.set_position_target(position=j1_ref)

        # Store the references in the queue
        queue.put_nowait(j1_ref)

        assert gazebo.run()

        # Check that the robot didn't move
        assert panda.joint_positions() == pytest.approx(q0, abs=np.deg2rad(1))

    for _ in range(2_000):

        # Continue applying the
        j1_ref = q0_j1 + next(q_j1)
        assert joint1.set_position_target(position=j1_ref)

        assert gazebo.run()

        # Compare the current position with the delayed target (popping it)
        assert joint1.position() == pytest.approx(queue.get_nowait(), abs=np.deg2rad(1))

        # Add the new target
        queue.put_nowait(j1_ref)
