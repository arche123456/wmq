from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rm_sentry_ai",
            executable="rm_sentry_ai_node",
            output="screen",
            parameters=[{
                "motor_groot2": True,
                "motor_groot2_port": 1669,
            }],
        )
    ])
