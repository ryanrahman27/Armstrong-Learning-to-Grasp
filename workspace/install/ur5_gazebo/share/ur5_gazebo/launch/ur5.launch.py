#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file')
    
    # Get package share directory
    pkg_ur5_gazebo = get_package_share_directory('ur5_gazebo')
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Spawn UR5 robot
    spawn_ur5 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'ur5',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(os.path.join(pkg_ur5_gazebo, 'urdf', 'ur5.urdf.xacro'), 'r').read()
        }]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Camera node for block detection
    camera_node = Node(
        package='ur5_gazebo',
        executable='camera_detector.py',
        name='camera_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Foxglove Bridge for web-based visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'use_sim_time': use_sim_time
        }]
    )

    # Environment visualizer for markers
    environment_visualizer = Node(
        package='ur5_gazebo',
        executable='environment_visualizer.py',
        name='environment_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(pkg_ur5_gazebo, 'worlds', 'pick_place_world.world'),
            description='Path to world file'
        ),
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_ur5,
        camera_node,
        foxglove_bridge,
        environment_visualizer
    ])