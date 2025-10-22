import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, action
from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler, Shutdown, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart


def generate_launch_description():

    root_dir = get_package_share_directory('rm_sentry_launch')
    launch_dir = os.path.join(root_dir, 'launch')

    bringup_cmd_group = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'livox_ros_driver.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'imu_filter.launch.py')),
        ),
        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'fast_lio.launch.py')),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localizer_launch.py')),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pc2scan.launch.py')),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'dll.launch.py')),
        # ),


        # 本文件作为烧饼的总启动文件， 启动相关程序W
        # livox - 雷达启动： 产生点云/陀螺仪
        # imu_f - 雷达陀螺仪滤波, 减少雷达震动带来的影响：输入陀螺仪，输出稳定陀螺仪
        # fast_lio - SLAM的一种实现, 用于实时定位：输入点云/稳定陀螺仪，输出lidar_init->lidar_now坐标系变换
        # nav2  - 用于导航：输入机器人相对map的坐标系关系，输出/cmd_vel控制信号
        # pc2scan - 3D点云转2D扫描，用于导航避障：输入点云, 输出平面扫描
        # dll - 直接重定位算法, 用于在长时间移动时, 纠正位置漂移：输入点云/稳定陀螺仪，输出map->foot_init坐标系变换
    ])

    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)

    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            on_exit=[Shutdown(reason='Some Node Exited Unexpectedly')]
        )
    ))

    return ld
