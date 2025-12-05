from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():
    launch_without_movement_file = os.path.join(
        os.path.dirname(__file__),
        'launch_without_movement.py'
    )
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_without_movement_file)
        ),
        Node(
            package='local_planning', executable='dynamic_window', name='dynamic_window',
            parameters=[],
            output='screen'),
    ])
    return ld
