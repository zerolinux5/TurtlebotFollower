from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='planner', executable='dynamic_window', name='dynamic_window',
            parameters=[
                {
                    'horizon': 5.0,
                    'speed_weight': 1.0,
                    'score_weight': 5.0,
                    'obstacle_weight': 0.075,
                    'v_max': 0.11,
                    'v_step': 10,
                    'w_min': -0.45,
                    'w_max': 0.45,
                    'w_step': 30,
                    'robot_radius': 0.22,
                }
            ],
            output='screen'),
    ])
    return ld
