import os.path
import xacro  # 一定要使用 sudo apt install ros-foxy-xacro安装 不要使用pip
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rm_nav2_bringup_dir = get_package_share_directory('rm_nav2_bringup')
    bringup_cmd_group = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rm_nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        ),

    ])
    xacro_file = os.path.join(get_package_share_directory("rm_gimbal_description"),
                              "urdf", "rm_2024_SENTRY_7_nav.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()  # URDF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)
    ld.add_action(robot_state_publisher_node)

    return ld
