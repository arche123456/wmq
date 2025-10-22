from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_serial_port',
            namespace='rm_serial_port',
            executable='rm_serial_port_cd',
            name='test'
        )
    ])
