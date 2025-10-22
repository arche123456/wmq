import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    filepath_map = os.path.abspath(os.path.join(
        get_package_share_directory('rm_sentry_launch'), "reloc_map", "RMUL.bt"))
    assert os.path.exists(filepath_map)
    return LaunchDescription([
        DeclareLaunchArgument('base_frame_id', default_value='robot_center'),
        DeclareLaunchArgument('odom_frame_id', default_value='foot_init'),
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('initial_x', default_value=str(2.6 + 0.9 / 2)),
        DeclareLaunchArgument('initial_y', default_value=str(-4.95 + 0.9 / 2)),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_a', default_value='0.0'),
        DeclareLaunchArgument('map_path', default_value=filepath_map),

        Node(
            package='dll',
            executable='dll_node',
            name='dll_node',
            # output='screen',
            # remappings=[('/dll_node/initial_pose', '/initialpose')],
            parameters=[{
                'in_cloud': '/cloud_registered',
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'global_frame_id': LaunchConfiguration('global_frame_id'),
                'update_rate': 30.0,
                'align_method': 3,
                'map_path': LaunchConfiguration('map_path'),
                'publish_point_cloud': True,
                'update_min_d': 2.0,
                'update_min_a': 1.0,
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_z': LaunchConfiguration('initial_z'),
                'initial_a': LaunchConfiguration('initial_a'),
                'update_min_time': 2.0,
                'use_imu': True
            }],
            remappings=[
                ('/imu/data', '/livox/imu_filter'),
                ('initial_pose', "/dll/reset_pose"),
            ],
        ),

        # Node(
        #     package='rviz',
        #     executable='rviz',
        #     name='rviz',
        #     output='screen'
        # ),
    ])
