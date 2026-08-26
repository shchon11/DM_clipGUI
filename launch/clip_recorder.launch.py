import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('clip_recorder'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='clip_recorder',
            executable='clip_recorder',
            name='clip_recorder',
            parameters=[params],
            output='screen',
        ),
    ])
