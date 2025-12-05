from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package='tracking', executable='yolo', name='yolo',
            remappings=[],
            output='screen'),
        Node(
            package='mediapipe_p', executable='mediapipe', name='mediapipe',
            remappings=[],
            output='log'),
        Node(
            package='robot_frame', executable='robot_frame_transform', name='robot_frame_transform',
            remappings=[],
            output='screen'),
        Node(
            package='gesture_recognition', executable='recognizer', name='recognizer',
            remappings=[],
            output='screen'),
    ])
    return ld
