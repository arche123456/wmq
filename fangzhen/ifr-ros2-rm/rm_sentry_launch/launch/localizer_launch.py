import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    # 定义localizer节点
    localizer_node = launch_ros.actions.Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": localizer_config_path.perform(launch.LaunchContext())}
        ],
    )
    service_request_args = """{
        pcd_path: "/home/ifr/ros2-sentry-lib/ifr-ros2-sentry-lib/pcd2pgm/pcd_saves/ncut.pcd",
        x: 0.0,
        y: 0.0,
        z: 0.0,
        yaw: 0.0,
        pitch: 0.0,
        roll: 3.14159
        }"""

    # 定义服务调用命令
    relocalize_service_call = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/localizer/relocalize',
            'interface/srv/Relocalize',
            # 注意使用YAML格式参数
            service_request_args,
        ],
        output='screen'
    )

    # 注册事件：当localizer_node启动后触发服务调用
    event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=localizer_node,
            on_start=[relocalize_service_call]
        )
    )

    return launch.LaunchDescription([
        localizer_node,
        # event_handler,
    ])
