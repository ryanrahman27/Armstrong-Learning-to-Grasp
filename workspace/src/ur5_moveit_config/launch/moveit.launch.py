#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_ur5_moveit_config = get_package_share_directory('ur5_moveit_config')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # Robot description parameter
    robot_description_content = open(os.path.join(pkg_ur5_moveit_config, '..', 'ur5_gazebo', 'urdf', 'ur5.urdf.xacro'), 'r').read()
    
    # MoveIt2 move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': open(os.path.join(pkg_ur5_moveit_config, 'config', 'ur5.srdf'), 'r').read()},
            os.path.join(pkg_ur5_moveit_config, 'config', 'kinematics.yaml'),
            os.path.join(pkg_ur5_moveit_config, 'config', 'joint_limits.yaml'),
            os.path.join(pkg_ur5_moveit_config, 'config', 'ompl_planning.yaml'),
            {'use_sim_time': use_sim_time},
            {'publish_robot_description_semantic': True},
            {'allow_trajectory_execution': True},
            {'max_safe_path_cost': 1},
            {'jiggle_fraction': 0.05},
            {'capabilities': ''},
            {'disable_capabilities': ''},
            {'enforce_boundary_constraints': True},
            {'enforce_joint_model_state_space': True}
        ]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
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
    
    # Static transform publisher for world frame
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_ur5_moveit_config, 'config', 'moveit.rviz'),
            description='Path to RViz config file'
        ),
        robot_state_publisher,
        joint_state_publisher,
        static_tf_publisher,
        move_group_node
    ])