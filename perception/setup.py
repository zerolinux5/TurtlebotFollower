from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/gesture_mlp.onnx', 'models/gesture_mlp.onnx.data']),
        ('share/' + package_name + '/configs', ['configs/bytetrack.yaml', 'configs/botsort.yaml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmagana',
    maintainer_email='maganazero5@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipe = mediapipe_p.mediapipe_p:main',
            'recognizer = gesture_recognition.gesture_recognition:main',
            'robot_frame_transform = robot_frame.robot_frame:main',
            'yolo = tracking.yolo:main'
        ],
    },
)
