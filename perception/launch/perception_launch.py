from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='perception', executable='yolo', name='yolo',
            remappings=[],
            output='screen'),
        Node(
            package='perception', executable='mediapipe', name='mediapipe',
            remappings=[],
            output='log'),
        Node(
            package='perception', executable='robot_frame_transform', name='robot_frame_transform',
            remappings=[],
            output='screen'),
        Node(
            package='perception', executable='recognizer', name='recognizer',
            remappings=[],
            output='screen'),
    ])
    return ld
