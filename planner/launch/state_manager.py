from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='state_manager', executable='manager', name='manager',
            remappings=[],
            output='screen'),
    ])
    return ld
