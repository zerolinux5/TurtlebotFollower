from setuptools import find_packages, setup

package_name = 'mediapipe_p'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            ],
    },
)
