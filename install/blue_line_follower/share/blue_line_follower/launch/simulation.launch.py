#!/usr/bin/env python3
"""
Launch file for the blue line follower robot in Gazebo
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package directories
    pkg_share = FindPackageShare('blue_line_follower').find('blue_line_follower')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'blue_line_track.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'line_follower_robot.urdf.xacro')
    models_path = os.path.join(pkg_share, 'models')
    controller_config = os.path.join(pkg_share, 'config', 'controller_config.yaml')
    
    # Set GAZEBO_MODEL_PATH to include our models
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        gazebo_model_path = models_path + ':' + gazebo_model_path
    else:
        gazebo_model_path = models_path
    
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )
    
    # Gazebo server launch
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client launch
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': True
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'line_follower_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Line Follower Node
    line_follower_node = Node(
        package='blue_line_follower',
        executable='line_follower_node',
        name='line_follower_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Controller Manager spawner for joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Controller Manager spawner for diff drive controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Controller Manager spawner for pusher controller
    pusher_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pusher_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # RViz pour visualiser le robot et les joints
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'robot_view.rviz')],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        set_gazebo_model_path,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        pusher_controller_spawner,
        rviz_node,
        line_follower_node
    ])
