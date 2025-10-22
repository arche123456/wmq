import os

from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('rm_aim_launch'), 'config', 'default.yaml')

    with open(params_file, 'r') as file:
        params = yaml.safe_load(file)

    def get_param(x): return params[x]['ros__parameters']

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
                    parameters=[get_param('camera')],
                    # extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                    package='rm_armor_finder',
                    plugin='rm_armor_finder::ArmorSaver',
                    name='rm_finder',
                    parameters=[get_param('rm_finder')],
                    # extra_arguments=[{'use_intra_process_comms': True}],
                    ),
        ],
        parameters=[params_file],
        output='screen',
    )
    return LaunchDescription([
        container,
    ])
