from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    launch_without_movement_file = os.path.join(
        os.path.dirname(__file__),
        'state_manager.py'
    )
    launch_movement_file = os.path.join(
        os.path.dirname(__file__),
        'movement.py'
    )
    ld = LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_without_movement_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_movement_file)
        ),
    ])
    return ld
