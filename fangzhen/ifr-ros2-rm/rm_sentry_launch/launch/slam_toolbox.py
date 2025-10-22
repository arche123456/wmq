from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['/home/ifr/ifr-ros2/ws-rm/src/rm_sentry_launch/config/slam_toolbox.yaml'],  # 配置文件路径
        ),
    ])
