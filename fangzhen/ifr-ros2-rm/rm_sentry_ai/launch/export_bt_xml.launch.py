from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rm_sentry_ai",
            executable="rm_sentry_ai_export_bt_xml",
            output="screen"
        )
    ])
