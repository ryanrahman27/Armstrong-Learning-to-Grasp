from setuptools import setup
import os
from glob import glob

package_name = 'ur5_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='UR5 Gazebo simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_detector = ur5_gazebo.camera_detector:main',
            'pick_place_controller = ur5_gazebo.pick_place_controller:main',
            'demo_recorder = ur5_gazebo.demo_recorder:main',
            'bc_policy_controller = ur5_gazebo.bc_policy_controller:main',
            'comparison_evaluator = ur5_gazebo.comparison_evaluator:main',
            'environment_visualizer = ur5_gazebo.environment_visualizer:main',
            'simple_pick_place = ur5_gazebo.simple_pick_place:main',
        ],
    },
)