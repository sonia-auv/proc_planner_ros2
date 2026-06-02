# proc_planner_ros2

The packages is the in-between the client and the control system for the prototype. It handles converting created waypoints from the client into series of transformations that the control can use to create a smooth trajectory the prototype can use to navigate.

---
## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_python`
* `rclpy`
* `trajectory_msgs`
* `geometry_msgs`
* `std_msgs`

### Sonia packages

* `sonia_common_ros2`

### External packages

* `setuptools`
* `numpy (1.24.4)`
* `scipy (1.10.1)`

---

## Node

* Name: `proc_planner`

---

## Registered Topics / Services / Actions

| Type   | Name                                | Direction  | Message/Service Type                               | Description                          |
| ------ | ----------------------------------- | -----------| -------------------------------------------------- | ------------------------------------ |
| Topi   | `/proc_planner/send_trajectory_list`| Published  | `trajectory_msgs/msg/MultiDOFJointTrajectoryPoint` | Messages containing a trajectory list|
| Topic  | `/proc_planner/is_waypoint_valid`   | Published  | `std_msgs/msg/Bool`                                | Message to validate the waypoint     |
| Topic  | `/proc_control/send_pose_array`     | Subscribed | `sonia_common_ros2/msg/PoseArray`                  | Messages of an array of waypoints    |
| Topic  | `/proc_control/current_target`      | Subscribed | `geometry_msgs/msg/Pose`                           | Message contains current position    |

---
## Build Instructions
To build the project, the following commands should be run directly from your ROS2 workspace.

```bash
colcon build --packages-select proc_planner_ros2 --symlink-install
source install/setup.bash
```

---

## Launch Instructions

### Default launch
The command launchs the mission server 

```bash
ros2 launch proc_planner_ros2 launch.py
```
---

## ROS Info

### Parameters

- `ts [double]`: Time step between waypoints.
- `max_depth [double]`: Maximum depth of the trajectory.
- `surface_warning [double]`: Distance below which the robot should stop.
- `low_speed.maximum_acceleration [double]`: Maximum acceleration for the low speed profile.
- `low_speed.maximum_velocity [double]`: Maximum velocity for the low speed profile.
- `low_speed.maximum_angular_rate [double]`: Maximum angular rate for the low speed profile.
- `normal_speed.maximum_acceleration [double]`: Maximum acceleration for the normal speed profile.
- `normal_speed.maximum_velocity [double]`: Maximum velocity for the normal speed profile.
- `normal_speed.maximum_angular_rate [double]`: Maximum angular rate for the normal speed profile.
- `high_speed.maximum_acceleration [double]`: Maximum acceleration for the high speed profile.
- `high_speed.maximum_velocity [double]`: Maximum velocity for the high speed profile.
- `high_speed.maximum_angular_rate [double]`: Maximum angular rate for the high speed profile.

---

## License

BSD 3-Clause License

## Support

If there are any issues, please open an issue and we will address it as soon as possible!

## Authors and acknowledgment

This package was created by the S.O.N.I.A. Software Team.
