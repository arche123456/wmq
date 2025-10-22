from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    params_file=os.path.join(get_package_share_directory('rm_fake_referee'),'config','default.yaml')
    assert os.path.exists(params_file), f"Not Found: {params_file}"
    with open(params_file,'r') as file:
        params=yaml.safe_load(file)
    def get_param(x, **kwargs) -> dict:
        data = {}
        if x in params:
            data.update(params[x]['ros__parameters'])
        data.update(kwargs)
        return data
    return LaunchDescription([
        Node(
            package='rm_fake_referee',
            executable='rm_fake_referee_rmul',
            name='rmul',
            namespace='/rm/fake/game',
            output='screen',
            parameters=[
            #   '/home/ifr/ifr-ros2/ws-rm/src/rm_fake_referee/config/default.yaml'
            ]
        ),
    ])

