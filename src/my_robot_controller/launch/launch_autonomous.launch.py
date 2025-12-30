#!/usr/bin/env python3
"""
Launch file pour le système autonome complet
============================================
Lance:
- Gazebo avec le monde
- Robot state publisher
- Controllers (diff_drive, arm, gripper)
- Le script autonome de pick and store

Usage:
    ros2 launch my_robot_controller launch_autonomous.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)

    # Process URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # World file
    world_file_path = os.path.join(pkg_share, 'worlds', 'my_world.world')

    # ========== GAZEBO ==========
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # ========== ROBOT STATE PUBLISHER ==========
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ========== SPAWN ROBOT ==========
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot',
            '-x', '0',
            '-y', '0',
            '-z', '0.3'
        ],
        output='screen'
    )

    # ========== CONTROLLERS ==========
    # Diff drive controller
    spawn_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )
    
    # Joint state broadcaster
    spawn_joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )
    
    # Arm controller
    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen"
    )
    
    # Gripper controller
    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen"
    )

    # ========== AUTONOMOUS SYSTEM (Delayed Start) ==========
    # Démarre après 10 secondes pour laisser le temps à tout de s'initialiser
    autonomous_system = TimerAction(
        period=15.0,  # Attendre 15 secondes
        actions=[
            Node(
                package='my_robot_controller',
                executable='autonomous_explorer.py',
                name='autonomous_explorer',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        spawn_diff_drive,
        spawn_joint_broad,
        spawn_arm,
        spawn_gripper,
        autonomous_system,
    ])
