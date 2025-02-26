# proc_planner_ros2

## Description

Package that manages the proc_planner in ROS2. It's main job is to take waypoints and convert them into a series of transformations that the control can use to create a smooth trajectory.

## Build

### Dependencies

This package is in python and has some package dependencies. To install run the following:

```bash
python3 -m pip install numpy scipy
```

It also depends on sonia_common_ros2.

### Build process

```bash
colcon build --packages-select proc_planner_ros2
```

## Usage

To run the proc planner node, use the following command:

```bash
ros2 launch proc_planner_ros2 launch.py
```

The Launch file loads the nessesary ros parameters.

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

### Topics

#### Subscriptions

- `/proc_control/send_pose_array [sonia_common_ros2.msg.PoseArray]` : Receive the array of waypoints.
- `/proc_control/current_target [geometry_msgs.msg.Pose]` : Receive the current position.

#### Publications

- `/proc_planner/send_trajectory_list [trajectory_msgs.msg.MultiDOFJointTrajectoryPoint]` : Send the trajectory list.
- `/proc_planner/is_waypoint_valid [std_msgs.msg.Bool]` : Send if the waypoint is valid.

## License

BSD 3-Clause License

## Support

If there are any issues, please open an issue and we will address it as soon as possible!

## Authors and acknowledgment

This package was created by the S.O.N.I.A. Software Team.
