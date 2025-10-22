import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('rm_armor_finder'), 'config', 'model.train.yaml')

    with open(params_file, 'r') as file:
        params = yaml.safe_load(file)

    def get_param(x): return params[x]['ros__parameters']

    model_train = Node(
        package="rm_armor_finder",
        executable="rm_armor_finder_test",
        parameters=[get_param('rm_armor_finder')],
        output="screen"
    )
    return LaunchDescription([
        model_train,
    ])
