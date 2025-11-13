from launch import LaunchDescription
from launch_ros.actions import Node

import launch_ros.actions

def generate_launch_description():
    ld = LaunchDescription([
        launch_ros.actions.Node(
            package='mediapipe_p', executable='mediapipe', name='mediapipe',
            remappings=[],
            output='log'),
        launch_ros.actions.Node(
            package='robot_frame', executable='robot_frame_transform', name='robot_frame_transform',
            remappings=[],
            output='screen'),
        launch_ros.actions.Node(
            package='gesture_recognition', executable='recognizer', name='recognizer',
            remappings=[],
            output='screen'),
        launch_ros.actions.Node(
            package='state_manager', executable='manager', name='manager',
            remappings=[],
            output='screen'),
    ])
    return ld
