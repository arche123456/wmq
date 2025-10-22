import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import xacro


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('rm_aim_launch'), 'config', 'default.yaml')

    container = ComposableNodeContainer(
        name='rm_aim',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='daheng_camera',
                    plugin='daheng_camera::CameraNode',
                    name='camera',
                    parameters=[params_file]),
                ComposableNode(
                    package='rm_armor_finder',
                    plugin='rm_armor_finder::ArmorFinder',
                    name='rm_finder',
                    parameters=[params_file]),
                ComposableNode(
                    package='rm_armor_processor',
                    plugin='rm_armor_processor::ArmorDirectNode',
                    name='rm_processor',
                    parameters=[params_file]),
                ComposableNode(
                    package='rm_serial_port',
                    plugin='rm_serial_port::SerialPortMotor',
                    name='rm_serial',
                    parameters=[params_file])
        ],
        output='screen',
    )

    xacro_file = os.path.join(get_package_share_directory("rm_gimbal_description"), "urdf", "rm_gimbal.urdf.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()  # URDF

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )
    return LaunchDescription([container, robot_state_publisher_node])
