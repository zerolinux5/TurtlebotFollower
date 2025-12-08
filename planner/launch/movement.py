from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='planner', executable='dynamic_window', name='dynamic_window',
            parameters=[
                {
                    'horizon': 5.0,
                    'speed_weight': 1.5,
                    'score_weight': 0.7,
                    'obstacle_weight': 0.1,
                    'v_max': 0.11,
                    'v_step': 10,
                    'w_min': -0.35,
                    'w_max': 0.35,
                    'v_step': 20,
                }
            ],
            output='screen'),
    ])
    return ld
