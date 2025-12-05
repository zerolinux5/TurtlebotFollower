from launch import LaunchDescription
from launch_ros.actions import Node

import launch_ros.actions

def generate_launch_description():
    ld = LaunchDescription([
        launch_ros.actions.Node(
            package='local_planning', executable='dynamic_window', name='dynamic_window',
            parameters=[
                {
                    'horizon': 5.0,
                    'speed_weight': 1.5,
                    'score_weight': 1.0,
                    'obstacle_weight': 0.1
                }
            ],
            output='screen'),
    ])
    return ld
