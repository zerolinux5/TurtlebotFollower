from launch import LaunchDescription
from launch_ros.actions import Node

import launch_ros.actions

def generate_launch_description():
    ld = LaunchDescription([
        launch_ros.actions.Node(
            package='local_movement', executable='local_movement', name='local_movement',
            parameters=[],
            output='screen'),
    ])
    return ld
