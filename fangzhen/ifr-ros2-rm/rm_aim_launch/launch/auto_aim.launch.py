import os

from ament_index_python.packages import get_package_share_directory
import xacro  # 一定要使用 sudo apt install ros-foxy-xacro安装 不要使用pip
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import yaml


def generate_launch_description():
    params_file_1 = os.path.join(get_package_share_directory('rm_aim_launch'), 'config', 'default.yaml')
    params_file_2 = os.path.join(get_package_share_directory('rm_aim_launch'), 'config', 'local.yaml')

    assert os.path.exists(params_file_1), f"Not Found: {params_file_1}"
    assert os.path.exists(params_file_2), f"Not Found: {params_file_2}, 你必须手动创建此文件, 即使文件是空的"

    with open(params_file_1, 'r') as file:
        params_1 = yaml.safe_load(file)
    with open(params_file_2, 'r') as file:
        params_2 = yaml.safe_load(file)

    def get_param(x, **kwargs) -> dict:
        data = {}
        if x in params_1:
            data.update(params_1[x]['ros__parameters'])
        if x in params_2:
            data.update(params_2[x]['ros__parameters'])
        data.update(kwargs)
        return data

    no_get_no_send_arg = DeclareLaunchArgument(
        'no_get_no_send',
        default_value='false',  # 默认为false
        description='If no get serial data, then do not send data'
    )
    no_get_no_send = LaunchConfiguration('no_get_no_send')

    container = ComposableNodeContainer(
        name='rm_aim',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # GDB调试命令:
        # prefix=['gdbserver localhost:3000'],
        # $: gdb -ex "target remote localhost:3000" -ex "continue"
        composable_node_descriptions=[
            #     ComposableNode(
            #         package='daheng_camera',
            #         plugin='daheng_camera::CameraNode',
            #         name='camera',
            #         parameters=[get_param('camera')],
            #         # extra_arguments=[{'use_intra_process_comms': True}],
            #     ),
            # ComposableNode(
            #         package='rm_armor_finder',
            #         plugin='rm_armor_finder::ArmorFinder',
            #         name='rm_finder',
            #         parameters=[get_param('rm_finder')],
            #         # extra_arguments=[{'use_intra_process_comms': True}],
            #         ),
            # ComposableNode(
            #         package='rm_armor_processor',
            #         plugin='rm_armor_processor::ArmorTrackerNode',
            #         name='rm_processor',
            #         parameters=[get_param('rm_processor')],
            #         # extra_arguments=[{'use_intra_process_comms': True}],
            #         ),
            ComposableNode(
                package='rm_serial_port',
                plugin='rm_serial_port::SerialPortMotor',
                name='rm_serial',
                parameters=[get_param('rm_serial', no_get_no_send=no_get_no_send)],
                # extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        # parameters=[params_file],
        output='screen',
        on_exit=Shutdown(),
    )

    robot_xacro_filename = get_param("rm_gimbal_description").get('robot', 'rm_gimbal')
    xacro_file = os.path.join(get_package_share_directory("rm_gimbal_description"),
                              "urdf", f"{robot_xacro_filename}.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()  # URDF

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )
    cpu_watcher_node = Node(
        package="ifr_watcher",
        executable="ifr_watcher_cpu",
        output="screen"
    )
    return LaunchDescription([
        no_get_no_send_arg,
        container,
        robot_state_publisher_node,
        cpu_watcher_node,
    ])
