import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    p4_dir = get_package_share_directory('guide_kobuki')
    config_rt = os.path.join(p4_dir, 'config', 'params.yaml')

    guide_cmd = Node(
        package='guide_kobuki',
        executable='guide_nav_main',
        name='guide_nav_main',
        output='screen',
        parameters=[config_rt]) 

    ld = LaunchDescription()

    ld.add_action(guide_cmd)

    return ld
