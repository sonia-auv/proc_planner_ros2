# Copyright 2025 SONIA AUV
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the SONIA AUV nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Proc Planner Node."""

import rclpy
from geometry_msgs.msg import Pose
from sonia_common_ros2.msg import PoseArray
from std_msgs.msg import Int8
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from proc_planner_ros2.trajectory_generator import TrajectoryGenerator



class ProcPlannerNode(Node):
    """Proc Planer Node."""

    def __init__(self):
        super().__init__('proc_planner_ros2')
        self._sub_mult_add_pose = self.create_subscription(
            PoseArray, '/proc_planner/send_pose_array', self._mult_add_pose_cb, 10
        )
        self.get_logger()
        self._sub_curr_target = self.create_subscription(Pose, '/proc_control/current_target', self._curr_target_cb, 10)

        self._pub_traj = self.create_publisher(MultiDOFJointTrajectoryPoint, '/proc_planner/send_trajectory_list', 10)
        self._pub_is_valid = self.create_publisher(Int8, '/proc_planner/is_waypoint_valid', 10)

        self._latest_curr_target = None

        self.declare_parameter('ts', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value))
        self.declare_parameter('max_depth', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value))
        self.declare_parameter(
            'surface_warning', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'low_speed.maximum_acceleration', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'low_speed.maximum_velocity', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'low_speed.maximum_angular_rate', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'normal_speed.maximum_acceleration', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'normal_speed.maximum_velocity', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'normal_speed.maximum_angular_rate', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'high_speed.maximum_acceleration', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'high_speed.maximum_velocity', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.declare_parameter(
            'high_speed.maximum_angular_rate', descriptor=ParameterDescriptor(type=rclpy.Parameter.Type.DOUBLE.value)
        )
        self.get_logger().info('Parameters declared')
        self._ros_params = {}

        for param in self.get_parameters([
            'ts',
            'max_depth',
            'surface_warning',
            'low_speed.maximum_acceleration',
            'low_speed.maximum_velocity',
            'low_speed.maximum_angular_rate',
            'normal_speed.maximum_acceleration',
            'normal_speed.maximum_velocity',
            'normal_speed.maximum_angular_rate',
            'high_speed.maximum_acceleration',
            'high_speed.maximum_velocity',
            'high_speed.maximum_angular_rate'
                ]):
            self._ros_params[param.name] = param.value

        for param in self._ros_params.items():
            self.get_logger().info(str(param) + ': ' + str(param))

    def _mult_add_pose_cb(self, msg: PoseArray):

        if self._latest_curr_target is None:
            return
        traj_gen = TrajectoryGenerator(msg, self._latest_curr_target, self._ros_params, self.get_logger())

        valid_msg = Int8()
        valid_msg.data = traj_gen.status
        self._pub_is_valid.publish(valid_msg)

        if traj_gen.status == traj_gen.TrajectoryStatus.RECEIVED_VALID_WAYPTS:
            traj_msg = traj_gen.compute()
            if traj_msg is not None:
                self._pub_traj.publish(traj_msg)

        self._latest_curr_target = None

    def _curr_target_cb(self, msg: Pose):
        self._latest_curr_target = msg
