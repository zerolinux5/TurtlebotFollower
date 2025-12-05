from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='local_planning', executable='dynamic_window', name='dynamic_window',
            parameters=[
                {
                    'horizon': 5.0,
                    'speed_weight': 1.5,
                    'score_weight': 0.7,
                    'obstacle_weight': 0.1
                }
            ],
            output='screen'),
    ])
    return ld
