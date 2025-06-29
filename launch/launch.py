from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("proc_planner_ros2"), "config", f"proc_planner_config.yaml"
    )
    print(config)
    return LaunchDescription([
        Node(
            package='proc_planner_ros2',
            executable='proc_planner_ros2',
            parameters=[config],
        ),
    ])