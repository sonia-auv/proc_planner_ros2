"""Test TrajectoryGenerator."""

# pylint: disable=no-member
from pathlib import Path
import numpy as np

import pytest
import rclpy
import yaml
from geometry_msgs.msg import Pose as geoPose
from geometry_msgs.msg import Transform, Twist
from rclpy.node import Node
from sonia_common_ros2.msg import Pose as soniaPose
from sonia_common_ros2.msg import PoseArray
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint as mdjTP

from proc_planner_ros2.trajectory_generator import TrajectoryGenerator


@pytest.fixture
def expected_traj_data():
    """
    Get the data from the yaml file for expected data.

    Returns
    -------
        MultiDOFJointTrajectoryPoint: The expected trajectory data.

    """
    with open(f"{str(Path(__file__).parent)}/data_move_x1.yaml", "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    traj = mdjTP()

    transforms_data = data["transforms"]

    for trans in transforms_data:
        msg = Transform()
        msg.translation.x = trans["translation"]["x"]
        msg.translation.y = trans["translation"]["y"]
        msg.translation.z = trans["translation"]["z"]
        msg.rotation.x = trans["rotation"]["x"]
        msg.rotation.y = trans["rotation"]["y"]
        msg.rotation.z = trans["rotation"]["z"]
        msg.rotation.w = trans["rotation"]["w"]
        traj.transforms.append(msg)

    velocity_data = data["velocities"]

    for vel in velocity_data:
        msg = Twist()
        msg.linear.x = vel["linear"]["x"]
        msg.linear.y = vel["linear"]["y"]
        msg.linear.z = vel["linear"]["z"]
        msg.angular.x = vel["angular"]["x"]
        msg.angular.y = vel["angular"]["y"]
        msg.angular.z = vel["angular"]["z"]
        traj.velocities.append(msg)

    acceleration_data = data["accelerations"]

    for acc in acceleration_data:
        msg = Twist()
        msg.linear.x = acc["linear"]["x"]
        msg.linear.y = acc["linear"]["y"]
        msg.linear.z = acc["linear"]["z"]
        msg.angular.x = acc["angular"]["x"]
        msg.angular.y = acc["angular"]["y"]
        msg.angular.z = acc["angular"]["z"]
        traj.accelerations.append(msg)

    traj.time_from_start.sec = data["time_from_start"]["secs"]
    traj.time_from_start.nanosec = data["time_from_start"]["nsecs"]
    return traj


@pytest.fixture
def generated_dummy_geoNode():
    """
    Generate current Position.

    Returns
    -------
        geometry_msgs.msg.Pose: The generated current position.

    """
    current_pose = geoPose()
    current_pose.position.x = 0.0
    current_pose.position.y = 0.0
    current_pose.position.z = 1.0
    current_pose.orientation.x = 0.0
    current_pose.orientation.y = 0.0
    current_pose.orientation.z = 0.0
    current_pose.orientation.w = 1.0
    return current_pose


@pytest.fixture
def generated_dummy_soniaNode():
    """
    Generate Sent waypoints.

    Returns
    -------
        sonia_msgs.msg.Pose: The generated sent waypoints.

    """
    move_pose1 = soniaPose()
    move_pose1.position.x = 1.0
    move_pose1.position.y = 0.0
    move_pose1.position.z = 0.0
    move_pose1.orientation.x = 0.0
    move_pose1.orientation.y = 0.0
    move_pose1.orientation.z = 0.0
    move_pose1.frame = move_pose1.FRAME_REL_POS_REL_ANG
    move_pose1.speed = move_pose1.SPEED_NORMAL
    move_pose1.fine = 0.0
    move_pose1.rotation = False

    move_pose2 = soniaPose()
    move_pose2.position.x = 0.0
    move_pose2.position.y = 1.0
    move_pose2.position.z = 0.0
    move_pose2.orientation.x = 0.0
    move_pose2.orientation.y = 0.0
    move_pose2.orientation.z = 0.0
    move_pose2.frame = move_pose2.FRAME_REL_POS_REL_ANG
    move_pose2.speed = move_pose2.SPEED_NORMAL
    move_pose2.fine = 0.0
    move_pose2.rotation = False

    multi_pose = PoseArray()
    multi_pose.poses.append(move_pose1)
    # multi_pose.poses.append(move_pose2)
    multi_pose.interpolation_method = multi_pose.INTERPOLATION_HERMITE
    return multi_pose


@pytest.fixture
def generated_ros_params():
    """
    Generate ROS parameters.

    Returns
    -------
        dict: The generated ROS parameters.

    """
    return {
        "ts": 0.1,
        "max_depth": 5.0,
        "surface_warning": 0.3,
        "low_speed.maximum_acceleration": 0.05,
        "low_speed.maximum_velocity": 0.2,
        "low_speed.maximum_angular_rate": 0.3,
        "normal_speed.maximum_acceleration": 0.1,
        "normal_speed.maximum_velocity": 0.5,
        "normal_speed.maximum_angular_rate": 0.5,
        "high_speed.maximum_acceleration": 0.15,
        "high_speed.maximum_velocity": 0.8,
        "high_speed.maximum_angular_rate": 0.8,
    }


def setup_function():
    """Initialize ROS."""
    rclpy.init()


def teardown_function():
    """Shutdown ROS."""
    rclpy.shutdown()


def test_traj_gen_init(generated_dummy_geoNode, generated_dummy_soniaNode, generated_ros_params):
    """
    Test that trajectory generator builds properly.

    Args:
    ----
        generated_dummy_geoNode (geometry_msgs.msg.Pose): Current Position
        generated_dummy_soniaNode (sonia_common_ros2.msg.PoseArray): Sent waypoints
        generated_ros_params (dict): ROS parameters

    """
    print()
    logger_node = Node("test_node")

    TrajectoryGenerator(
        generated_dummy_soniaNode, generated_dummy_geoNode, generated_ros_params, logger_node.get_logger()
    )


def test_traj_gen_compute(expected_traj_data, generated_dummy_geoNode, generated_dummy_soniaNode, generated_ros_params):
    """
    Test to see if the trajectory generator computes the correct trajectory.

    Args:
    ----
        expected_traj_data (trajectory_msgs.msg.MultiDOFJointTrajectoryPoint): The expected trajectory data.
        generated_dummy_geoNode (geometry_msgs.msg.Pose): Current Position
        generated_dummy_soniaNode (sonia_common_ros2.msg.PoseArray): Sent waypoints
        generated_ros_params (dict): ROS parameters

    """
    print()
    logger_node = Node("test_node")

    traj_gen = TrajectoryGenerator(
        generated_dummy_soniaNode, generated_dummy_geoNode, generated_ros_params, logger_node.get_logger()
    )
    actual_out = traj_gen.compute()
    assert len(actual_out.transforms) == len(expected_traj_data.transforms)
    assert len(actual_out.velocities) == len(expected_traj_data.velocities)
    assert len(actual_out.accelerations) == len(expected_traj_data.accelerations)

    for val, ref in zip(actual_out.transforms, expected_traj_data.transforms):
        assert np.round(val.translation.x, 10) == np.round(ref.translation.x, 10)
        assert np.round(val.translation.y, 10) == np.round(ref.translation.y, 10)
        assert np.round(val.translation.z, 10) == np.round(ref.translation.z, 10)
        assert np.round(val.rotation.x, 10) == np.round(ref.rotation.x, 10)
        assert np.round(val.rotation.y, 10) == np.round(ref.rotation.y, 10)
        assert np.round(val.rotation.z, 10) == np.round(ref.rotation.z, 10)
        assert np.round(val.rotation.w, 10) == np.round(ref.rotation.w, 10)

    for val, ref in zip(actual_out.velocities, expected_traj_data.velocities):
        assert np.round(val.linear.x, 10) == np.round(ref.linear.x, 10)
        assert np.round(val.linear.y, 10) == np.round(ref.linear.y, 10)
        assert np.round(val.linear.z, 10) == np.round(ref.linear.z, 10)
        assert np.round(val.angular.x, 10) == np.round(ref.angular.x, 10)
        assert np.round(val.angular.y, 10) == np.round(ref.angular.y, 10)
        assert np.round(val.angular.z, 10) == np.round(ref.angular.z, 10)

    for val, ref in zip(actual_out.accelerations, expected_traj_data.accelerations):
        assert np.round(val.linear.x, 10) == np.round(ref.linear.x, 10)
        assert np.round(val.linear.y, 10) == np.round(ref.linear.y, 10)
        assert np.round(val.linear.z, 10) == np.round(ref.linear.z, 10)
        assert np.round(val.angular.x, 10) == np.round(ref.angular.x, 10)
        assert np.round(val.angular.y, 10) == np.round(ref.angular.y, 10)
        assert np.round(val.angular.z, 10) == np.round(ref.angular.z, 10)
