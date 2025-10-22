from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from math import pi


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ifr_pointcloud_to_laserscan', executable='ifr_pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/cloud_registered_body'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'robot_center',
                'transform_tolerance': 0.01,
                'min_height': -0.2,
                'max_height': 0.5,
                'angle_min': -pi,
                'angle_max': pi,
                'angle_increment': pi / 360,
                'scan_time': 0.3333,
                'range_min': 0.55,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
