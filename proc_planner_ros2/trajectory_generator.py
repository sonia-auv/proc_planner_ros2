"""Trajectory Generator module."""

from enum import IntEnum
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose as geoPose, PoseStamped
from geometry_msgs.msg import Transform, Twist
from sonia_common_ros2.msg import Pose as soniaPose
from sonia_common_ros2.msg import PoseArray
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from scipy.spatial.transform import Rotation, Slerp
from scipy.interpolate import PchipInterpolator, CubicSpline
from scipy.ndimage import map_coordinates

from builtin_interfaces.msg import Time
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryGenerator:
    """
    Calculation block that will generate the trajectory given the input waypoints.

    NOTE: ALL QUATERNIONS ARE IN THE FORMAT X,Y,Z,W
    """

    class TrajectoryStatus(IntEnum):
        """
        Status codes for the Trajectory Generator.

        Arguments:
        ---------
            IntEnum -- zero means okay, positive means warning, negative means error.

        """

        WARN_AUV_MAY_SURFACE = 1
        RECEIVED_VALID_WAYPTS = 0
        ERR_INVALID_INTERP_METHOD = -1
        ERR_INVALID_FRAME_REF = -2
        ERR_INVALID_SPEED_PARAM = -3
        ERR_RADIUS_TOO_LARGE = -4
        ERR_INVALID_CURR_POS = -5
        ERR_TRAJ_EXCEDED_MAX_DEPTH = -6

    def __init__(
        self,
        multi_add_pose_msg: PoseArray,
        current_position_msg: geoPose,
        ros_params: dict,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ):

        self._logger = logger

        self._curr_pos_offset = 2

        self._status = self.TrajectoryStatus.RECEIVED_VALID_WAYPTS

        self._mult_add_pose_msg = multi_add_pose_msg

        self._ros_param = ros_params

        self._current_position_msg = current_position_msg

        if self._mult_add_pose_msg.poses[-1].fine != 0:
            self._mult_add_pose_msg.poses[-1].fine = 0.0
            self._logger.warning(" proc_planner: last waypont must have fine param set to 0")

        # supplementary points for rounding
        supp_point = 0
        for pose in self._mult_add_pose_msg.poses:
            if pose.fine != 0:
                supp_point = supp_point + 1

        # number of waypoints + support_points + current position + initial condition
        self._n = len(self._mult_add_pose_msg.poses) + supp_point + self._curr_pos_offset + 1

        self._point_list = np.zeros((self._n, 3), np.float64)
        self._quat_list = np.zeros((self._n, 4), np.float64)
        self._time_list = np.zeros((self._n, 1), np.float64)
        self._course_list = np.zeros((1, self._n), np.float64)
        self._speed_list = np.zeros((1, self._n), np.float64)

        # Find initial waypoint
        if not self._get_initial_waypoint(self._current_position_msg):
            self._status = self.TrajectoryStatus.ERR_INVALID_CURR_POS
            self._logger.info("initial waypoint not received")

        # Validate the interpolation Method
        if not bool(self._interp_strategy(0, 0, 0, True)):
            self._status = self.TrajectoryStatus.ERR_INVALID_INTERP_METHOD
            self._logger.info("Interpolation strategy is not recognized")

        self._process_waypoints()

        # Calculate the time between each waypoint
        if self._status == self.TrajectoryStatus.RECEIVED_VALID_WAYPTS and bool(self._compute_time_arrival()):
            self._nb_points = int(np.round(self._time_list[-1][0] / float(self._ros_param["ts"])))
        else:
            self._nb_points = 1

        # Validate max depth
        if np.max(self._point_list[:, 2]) > self._ros_param["max_depth"]:
            self._status = self.TrajectoryStatus.ERR_TRAJ_EXCEDED_MAX_DEPTH
            self._logger.warning("Trajectory exceeded max depth")

        # TODO: Validate with Missions
        # Validate if sub will breach water surface
        # if np.min(self._point_list[:, 2]) < 0:
        #     self._status = self.TrajectoryStatus.ERR_TRAJ_BREACHES_WTR_SFC
        #     self._logger.warning("Trajectory breaches water surface")

        # Define the length of the trajectory
        self._traj_position = np.zeros((self._nb_points, 3), np.float64)
        self._traj_quat = np.zeros((self._nb_points, 4), np.float64)
        self._traj_body_velocity = np.zeros((self._nb_points, 3), np.float64)
        self._traj_angular_rates = np.zeros((self._nb_points, 3), np.float64)
        self._traj_linear_acceleration = np.zeros((self._nb_points, 3), np.float64)
        self._traj_anglular_acceleration = np.zeros((self._nb_points, 3), np.float64)

    @property
    def status(self) -> int:
        """
        Status of Generator.

        Returns
        -------
            int: Status code.

        """
        return self._status.value

    def compute(self) -> Optional[MultiDOFJointTrajectoryPoint]:
        """
        Compute the trajectory.

        Returns
        -------
            Optional[MultiDOFJointTrajectoryPoint]: The computed trajectory.

        """
        if self._status == self.TrajectoryStatus.RECEIVED_VALID_WAYPTS:
            self._interpolate_waypoints()

            return self._send_waypoints()
        return None

    def _get_initial_waypoint(self, curr_pos_msg: geoPose) -> bool:
        # Fill the lists
        self._point_list[0, :] = np.array([curr_pos_msg.position.x, curr_pos_msg.position.y, curr_pos_msg.position.z])

        self._quat_list[0, :] = np.array(
            [
                curr_pos_msg.orientation.x,
                curr_pos_msg.orientation.y,
                curr_pos_msg.orientation.z,
                curr_pos_msg.orientation.w,
            ]
        )

        self._time_list[0, :] = 0
        self._speed_list[0, 0] = 0
        eul = np.rad2deg(Rotation.from_quat(self._quat_list[0, :]).as_euler("ZYX"))
        self._course_list[0, 0] = eul[0]

        # Copy points to the second value to force inital acceleration to zero
        self._point_list[1, :] = self._point_list[0, :]
        self._quat_list[1, :] = self._quat_list[0, :]
        self._time_list[1, :] = float(self._ros_param["ts"])
        self._course_list[0, 1] = eul[0]
        self._speed_list[0, 1] = self._speed_list[0, 0]
        return True

    def _interp_strategy(
        self,
        time_list: np.ndarray,
        point_list: np.ndarray,
        sample: np.ndarray,
        verif: bool,
    ) -> int:
        traj_list = None
        if self._mult_add_pose_msg.interpolation_method == PoseArray.INTERPOLATION_HERMITE:
            if not verif:
                traj_list = PchipInterpolator(time_list, point_list)(sample)
            else:
                traj_list = 1
        elif self._mult_add_pose_msg.interpolation_method == PoseArray.INTERPOLATION_V5CUBIC:
            if not verif:
                traj_list = CubicSpline(time_list, point_list)(sample)
            else:
                traj_list = 1
        elif self._mult_add_pose_msg.interpolation_method == PoseArray.INTERPOLATION_SPLINE:
            if not verif:
                sample_indices = (sample - time_list[0]) / (time_list[-1] - time_list[0])
                traj_list = map_coordinates(point_list, sample_indices, order=3, mode="nearest")
            else:
                traj_list = 1
        else:
            traj_list = 0
        return traj_list

    def _process_waypoints(self) -> None:
        for i, pose in enumerate(self._mult_add_pose_msg.poses):
            q = Rotation.from_euler(
                "zyx",
                np.deg2rad([pose.orientation.z, pose.orientation.y, pose.orientation.x]),
            ).as_quat()
            p = np.array([pose.position.x, pose.position.y, pose.position.z])

            frame = pose.frame

            if frame == soniaPose.FRAME_ABS_POS_ABS_ANG:
                self._quat_list[i + self._curr_pos_offset, :] = self._check_continuity(
                    q, self._quat_list[i + self._curr_pos_offset - 1, :]
                )
                self._point_list[i + self._curr_pos_offset, :] = p
            elif frame == soniaPose.FRAME_REL_POS_REL_ANG:
                self._quat_list[i + self._curr_pos_offset, :] = (
                    Rotation.from_quat(self._quat_list[i + self._curr_pos_offset - 1, :]) * Rotation.from_quat(q)
                ).as_quat()
                self._point_list[i + self._curr_pos_offset, :] = self._point_list[
                    i + self._curr_pos_offset - 1, :
                ] + Rotation.from_quat(self._quat_list[i + self._curr_pos_offset - 1, :]).apply(p)
            elif frame == soniaPose.FRAME_REL_POS_ABS_ANG:
                self._quat_list[i + self._curr_pos_offset, :] = self._check_continuity(
                    q, self._quat_list[i + self._curr_pos_offset - 1, :]
                )
                self._point_list[i + self._curr_pos_offset, :] = self._point_list[
                    i + self._curr_pos_offset - 1, :
                ] + Rotation.from_quat(self._quat_list[i + self._curr_pos_offset - 1, :]).apply(p)
            elif frame == soniaPose.FRAME_ABS_POS_REL_ANG:
                self._quat_list[i + self._curr_pos_offset, :] = (
                    Rotation.from_quat(self._quat_list[i + self._curr_pos_offset - 1, :]) * Rotation.from_quat(q)
                ).as_quat()
                self._point_list[i + self._curr_pos_offset, :] = p
            elif frame == soniaPose.FRAME_ABS_DEPTH_REL_OTHER:
                self._quat_list[i + self._curr_pos_offset, :] = (
                    Rotation.from_quat(self._quat_list[i + self._curr_pos_offset - 1, :]) * Rotation.from_quat(q)
                ).as_quat()
                self._point_list[i + self._curr_pos_offset, :] = self._point_list[
                    i + self._curr_pos_offset - 1, :
                ] + Rotation.from_quat(self._quat_list[i + self._curr_pos_offset - 1, :]).apply(p)
                self._point_list[i + self._curr_pos_offset, 2] = p[2]
            else:
                # TODO: Obstical stuff
                return

            self._speed_list[0, i + self._curr_pos_offset] = pose.speed

            self._course_list[0, i + self._curr_pos_offset] = self._get_course_angle(
                self._quat_list[i + self._curr_pos_offset, :]
            )

            if i > 0 and self._mult_add_pose_msg.poses[i - 1].fine != 0:
                valid, p01, p12 = self._inscribed_circles(i + self._curr_pos_offset)

                if not valid:
                    self._status = self.TrajectoryStatus.ERR_RADIUS_TOO_LARGE
                    self._logger.info("Circle radius is to large.")
                    return

                # Declare waypoints
                self._point_list[i + self._curr_pos_offset + 1, :] = self._point_list[i + self._curr_pos_offset, :]
                self._point_list[i + self._curr_pos_offset, :] = p12
                self._point_list[i + self._curr_pos_offset - 1, :] = p01

                self._quat_list[i + self._curr_pos_offset + 1, :] = self._quat_list[i + self._curr_pos_offset, :]
                self._quat_list[i + self._curr_pos_offset, :] = self._quat_list[i + self._curr_pos_offset - 1, :]
                self._quat_list[i + self._curr_pos_offset - 1, :] = self._quat_list[i + self._curr_pos_offset - 2, :]

                self._speed_list[0, i + self._curr_pos_offset + 1] = self._speed_list[0, i + self._curr_pos_offset]
                self._speed_list[0, i + self._curr_pos_offset] = self._speed_list[0, i + self._curr_pos_offset - 1]

                self._course_list[0, i + self._curr_pos_offset + 1] = self._course_list[0, i + self._curr_pos_offset]
                self._course_list[0, i + self._curr_pos_offset] = self._course_list[0, i + self._curr_pos_offset - 1]
                self._course_list[0, i + self._curr_pos_offset - 1] = self._course_list[
                    0, i + self._curr_pos_offset - 2
                ]

                self._curr_pos_offset += 1

            self._point_list[-1, :] = self._point_list[-2, :]
            self._quat_list[-1, :] = self._quat_list[-2, :]
            self._course_list[0, -1] = self._course_list[0, -2]
            self._speed_list[0, -1] = self._speed_list[0, -2]

    def _inscribed_circles(self, i) -> Tuple[bool, float, float]:
        status = False
        p01 = 0
        p12 = 0

        r_bar = self._mult_add_pose_msg.poses[i - self._curr_pos_offset - 1].fine
        if r_bar > 0:
            p0 = self._point_list[i - 2, :]
            p1 = self._point_list[i - 1, :]
            p2 = self._point_list[i, :]

            v02 = p2 - p0
            v01 = p1 - p0
            v12 = p2 - p1

            a = np.linalg.norm(v02)
            b = np.linalg.norm(v01)
            c = np.linalg.norm(v12)

            alpha_1 = (1 / 2) * np.arccos((-(a**2) + b**2 + c**2) / (2 * b * c))

            r_1 = r_bar / np.tan(alpha_1)

            if r_1 < b or r_1 < c:
                p01 = p0 + (v01 / b) * (b - r_1)
                p12 = p1 + (v12 / c) * r_1
                status = True
            else:
                p01 = np.zeros((1, 3))
                p12 = np.zeros((1, 3))
                status = False
        else:
            p01 = self._point_list[i - 1, :]
            p12 = self._point_list[i - 1, :]
            status = True

        return status, p01, p12

    def _compute_time_arrival(self) -> bool:
        for i in range(1, self._n):
            amax = 0
            vlmax = 0
            vamax = 0
            speed = self._speed_list[0, i]
            if speed == 0:
                amax = float(self._ros_param["normal_speed.maximum_acceleration"])
                vlmax = float(self._ros_param["normal_speed.maximum_velocity"])
                vamax = float(self._ros_param["normal_speed.maximum_angular_rate"])
            elif speed == 1:
                amax = self._ros_param["high_speed.maximum_acceleration"]
                vlmax = self._ros_param["high_speed.maximum_velocity"]
                vamax = self._ros_param["high_speed.maximum_angular_rate"]
            elif speed == 2:
                amax = self._ros_param["low_speed.maximum_acceleration"]
                vlmax = self._ros_param["low_speed.maximum_velocity"]
                vamax = self._ros_param["low_speed.maximum_angular_rate"]
            else:
                self._logger.info("Speed not recognized")
                return False

            d = np.linalg.norm(self._point_list[i, :] - self._point_list[i - 1, :])

            tl = (4 * np.sqrt(3 * d)) / (3 * np.sqrt(amax))

            vl = (amax * tl) / 4

            if vl > vlmax:
                tl = (4 * d) / (3 * vlmax)

            q_rel = (
                Rotation.from_quat(np.conj(self._quat_list[i - 1, :])).inv() * Rotation.from_quat(self._quat_list[i, :])
            ).as_quat()
            travel_angle = 2 * np.arctan2(np.linalg.norm(q_rel[1:3]), q_rel[0])
            ta = travel_angle / vamax

            tmax = np.max(np.array([tl, ta, float(self._ros_param["ts"])]))

            t_residual = np.mod(tmax, float(self._ros_param["ts"]))
            if t_residual > 0:
                tmax = tmax + (float(self._ros_param["ts"]) - t_residual)

            self._time_list[i, 0] = self._time_list[i - 1, 0] + tmax
        return True


    def _interpolate_quaternions(self, t):
        key_times = self._time_list[:, 0]
        key_rots = Rotation.from_quat(self._quat_list)
        slerp = Slerp(key_times, key_rots)
        interp_rots = slerp(t)
        return interp_rots.as_quat()

    def _interpolate_waypoints(self):

        t = np.linspace(
            start=float(self._ros_param["ts"]),
            stop=np.round(self._time_list[-1, 0], 1),
            num=self._traj_position[:, 0].shape[0]
        )

        self._traj_position[:, 0] = self._interp_strategy(self._time_list[:, 0], self._point_list[:, 0], t, False).T
        self._traj_position[:, 1] = self._interp_strategy(self._time_list[:, 0], self._point_list[:, 1], t, False).T
        self._traj_position[:, 2] = (PchipInterpolator(self._time_list[:, 0], self._point_list[:, 2])(t)).T

        self._traj_body_velocity[:, 0] = np.append([0], np.diff(self._traj_position[:, 0]))
        self._traj_body_velocity[:, 1] = np.append([0], np.diff(self._traj_position[:, 1]))
        self._traj_body_velocity[:, 2] = np.append([0], np.diff(self._traj_position[:, 2]))

        self._traj_linear_acceleration[:, 0] = np.append([0], np.diff(self._traj_body_velocity[:, 0]))
        self._traj_linear_acceleration[:, 1] = np.append([0], np.diff(self._traj_body_velocity[:, 1]))
        self._traj_linear_acceleration[:, 2] = np.append([0], np.diff(self._traj_body_velocity[:, 2]))

        # interp_quats = self._interpolate_quaternions(t)
        # self._traj_quat[:, :] = interp_quats

        self._traj_quat[:, 0] = PchipInterpolator(self._time_list[:, 0], self._quat_list[:, 0])(t).T
        self._traj_quat[:, 1] = PchipInterpolator(self._time_list[:, 0], self._quat_list[:, 1])(t).T
        self._traj_quat[:, 2] = PchipInterpolator(self._time_list[:, 0], self._quat_list[:, 2])(t).T
        self._traj_quat[:, 3] = PchipInterpolator(self._time_list[:, 0], self._quat_list[:, 3])(t).T

        qdot = np.zeros((self._nb_points, 4))
        qdot[:, 0] = np.append([0], np.diff(self._traj_quat[:, 0]))
        qdot[:, 1] = np.append([0], np.diff(self._traj_quat[:, 1]))
        qdot[:, 2] = np.append([0], np.diff(self._traj_quat[:, 2]))
        qdot[:, 3] = np.append([0], np.diff(self._traj_quat[:, 3]))

        for i in range(self._nb_points):
            self._traj_quat[i, :] = self._traj_quat[i, :] / np.linalg.norm(self._traj_quat[i, :])

            if i > 0 and np.dot(self._traj_quat[i - 1, :], self._traj_quat[i, :]) < 0:
                self._traj_quat[i, :] = -self._traj_quat[i, :]

            self._traj_body_velocity[i, :] = Rotation.from_quat(self._traj_quat[i, :]).apply(
                self._traj_body_velocity[i, :]
            )

            self._traj_angular_rates[i, :] = self._quat_to_angular_rates(self._traj_quat[i, :], qdot[i, :])

            self._traj_angular_rates[i, :] = -self._traj_angular_rates[i, :]

            # self._traj_linear_acceleration[i, :] = Rotation.from_quat(self._traj_quat[i, :]).apply(
            #     self._traj_linear_acceleration[i, :]
            # )

        self._traj_anglular_acceleration[:, 0] = np.append([0], np.diff(self._traj_angular_rates[:, 0]))
        self._traj_anglular_acceleration[:, 1] = np.append([0], np.diff(self._traj_angular_rates[:, 1]))
        self._traj_anglular_acceleration[:, 2] = np.append([0], np.diff(self._traj_angular_rates[:, 2]))

    def _send_waypoints(self):
        traj_msg = MultiDOFJointTrajectoryPoint()

        for i in range(self._nb_points):
            transform = Transform()
            transform.translation.x = float(self._traj_position[i, 0])
            transform.translation.y = float(self._traj_position[i, 1])
            transform.translation.z = float(self._traj_position[i, 2])

            transform.rotation.w = float(self._traj_quat[i, 3])
            transform.rotation.x = float(self._traj_quat[i, 0])
            transform.rotation.y = float(self._traj_quat[i, 1])
            transform.rotation.z = float(self._traj_quat[i, 2])

            velocities = Twist()
            velocities.linear.x = float(self._traj_body_velocity[i, 0])
            velocities.linear.y = float(self._traj_body_velocity[i, 1])
            velocities.linear.z = float(self._traj_body_velocity[i, 2])
            velocities.angular.x = float(self._traj_angular_rates[i, 0])
            velocities.angular.y = float(self._traj_angular_rates[i, 1])
            velocities.angular.z = float(self._traj_angular_rates[i, 2])

            accelerations = Twist()
            accelerations.linear.x = float(self._traj_linear_acceleration[i, 0])
            accelerations.linear.y = float(self._traj_linear_acceleration[i, 1])
            accelerations.linear.z = float(self._traj_linear_acceleration[i, 2])
            accelerations.angular.x = float(self._traj_anglular_acceleration[i, 0])
            accelerations.angular.y = float(self._traj_anglular_acceleration[i, 1])
            accelerations.angular.z = float(self._traj_anglular_acceleration[i, 2])

            # pylint: disable=no-member
            traj_msg.transforms.append(transform)
            traj_msg.velocities.append(velocities)
            traj_msg.accelerations.append(accelerations)
            # pylint: enable=no-member
        return traj_msg

    @staticmethod
    def _check_continuity(q: np.ndarray, qk: np.ndarray) -> np.ndarray:
        if np.dot(qk, q) < 0:
            return -q
        return q

    @staticmethod
    def quat_multiply(q1, q2):
        """Multiplies two quaternions q1 and q2 (both in w, x, y, z format)."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    @staticmethod
    def _get_course_angle(q):
        eul = Rotation.from_quat(q).as_euler("ZYX", degrees=True)
        if eul[0] < 0:
            eul[0] += 360

        return eul[0]

    @staticmethod
    def _quat_to_angular_rates(q, q_dot):
        """
        Compute angular rates from quaternion and its time derivative (qDot).

        Arguments:
        ----------
            q -- quaternion (4, ) array in [x, y, z, w] format.
            q_dot -- quaternion time derivative (4, ) array in [x, y, z, w] format.

        Returns
        -------
            angularRates -- angular velocity (3, ) array.

        """
        # Skew matrix function (equivalent to skew3)
        def skew3(u):
            """Create the skew-symmetric matrix for vector u."""
            return np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])

        # Linearize E2 (equivalent to liniairzeE2)
        def liniairze_e2(q):
            """Compute the linearized E2 matrix."""
            q_vec = q[1:4]
            return 0.5 * np.vstack((-q_vec, q[0] * np.eye(3) + skew3(q_vec)))

        # Compute E2 and its inverse
        e2 = liniairze_e2(q)
        inv_e2 = np.linalg.pinv(e2)  # Pseudo-inverse of E2

        # Calculate the angular rates
        angular_rates = -(inv_e2 @ q_dot)  # Matrix multiplication

        return angular_rates


    def to_nav_path(self, frame_id: str = "map"):
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        # path_msg.header.stamp = self._logger.get_clock().now().to_msg()

        for i in range(self._nb_points):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(self._traj_position[i, 0])
            pose.pose.position.y = float(self._traj_position[i, 1])
            pose.pose.position.z = float(self._traj_position[i, 2])

            pose.pose.orientation.x = float(self._traj_quat[i, 0])
            pose.pose.orientation.y = float(self._traj_quat[i, 1])
            pose.pose.orientation.z = float(self._traj_quat[i, 2])
            pose.pose.orientation.w = float(self._traj_quat[i, 3])

            path_msg.poses.append(pose)

        return path_msg
    
    
    def to_orientation_markers(self, frame_id="map") -> MarkerArray:
        marker_array = MarkerArray()
        for i in range(0, self._nb_points, 5):  # Downsample for clarity
            marker = Marker()
            marker.header.frame_id = frame_id
            # marker.header.stamp = self._logger.get_clock().now().to_msg()
            marker.ns = "traj_orientation"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = self._traj_position[i, 0]
            marker.pose.position.y = self._traj_position[i, 1]
            marker.pose.position.z = self._traj_position[i, 2]
            marker.pose.orientation.x = self._traj_quat[i, 0]
            marker.pose.orientation.y = self._traj_quat[i, 1]
            marker.pose.orientation.z = self._traj_quat[i, 2]
            marker.pose.orientation.w = self._traj_quat[i, 3]
            marker.scale.x = 0.3  # shaft length
            marker.scale.y = 0.05  # shaft diameter
            marker.scale.z = 0.05  # head diameter
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        return marker_array
