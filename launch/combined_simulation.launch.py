#!/usr/bin/env python3
"""
Launch file combiné pour les deux projets:
- my_robot_controller: spawné à la position d'origine (0, 0)
- blue_line_follower: spawné décalé (+50, 0)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    
    # ====== PACKAGE DIRECTORIES ======
    my_robot_pkg = get_package_share_directory('my_robot_controller')
    blue_line_pkg = FindPackageShare('blue_line_follower').find('blue_line_follower')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # ====== WORLD FILE ======
    # Utiliser le monde combiné
    world_file = os.path.join('/home/wayay/mixed/worlds', 'combined_world.world')
    
    # ====== GAZEBO MODEL PATH ======
    # Ajouter les modèles du projet blue_line_follower
    models_path = os.path.join(blue_line_pkg, 'models')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        gazebo_model_path = models_path + ':' + gazebo_model_path
    else:
        gazebo_model_path = models_path
    
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )
    
    # ====== GAZEBO LAUNCH ======
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # ====== MY_ROBOT_CONTROLLER (Position originale: 0, 0) ======
    
    # Robot URDF pour my_robot_controller
    my_robot_xacro = os.path.join(my_robot_pkg, 'description', 'robot.urdf.xacro')
    my_robot_description_config = xacro.process_file(my_robot_xacro)
    
    # Robot State Publisher pour my_robot
    my_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='my_bot',
        output='screen',
        parameters=[{
            'robot_description': my_robot_description_config.toxml(),
            'use_sim_time': True
        }]
    )
    
    # Spawn my_robot à la position d'origine
    spawn_my_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_my_robot',
        arguments=[
            '-topic', '/my_bot/robot_description',
            '-entity', 'my_bot',
            '-x', '0',
            '-y', '0',
            '-z', '0.4',
            '-robot_namespace', 'my_bot'
        ],
        output='screen'
    )
    
    # Controllers pour my_robot
    spawn_my_robot_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", '--controller-manager', '/my_bot/controller_manager'],
        output="screen"
    )
    
    spawn_my_robot_joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", '--controller-manager', '/my_bot/controller_manager'],
        output="screen"
    )
    
    spawn_my_robot_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", '--controller-manager', '/my_bot/controller_manager'],
        output="screen"
    )
    
    spawn_my_robot_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", '--controller-manager', '/my_bot/controller_manager'],
        output="screen"
    )
    
    # EKF pour my_robot
    ekf_config = os.path.join(my_robot_pkg, 'config', 'ekf_params.yaml')
    my_robot_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='my_bot',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odom/filtered')]
    )
    
    # ====== BLUE_LINE_FOLLOWER (Position décalée: +50, 0) ======
    
    # Robot URDF pour blue_line_follower
    blue_robot_urdf = os.path.join(blue_line_pkg, 'urdf', 'line_follower_robot.urdf.xacro')
    blue_robot_description_config = xacro.process_file(blue_robot_urdf)
    
    # Robot State Publisher pour blue_line robot
    blue_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='line_follower',
        output='screen',
        parameters=[{
            'robot_description': blue_robot_description_config.toxml(),
            'use_sim_time': True
        }]
    )
    
    # Spawn blue_line_follower robot à la position décalée (+50, 0)
    spawn_blue_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_blue_robot',
        arguments=[
            '-entity', 'line_follower_robot',
            '-topic', '/line_follower/robot_description',
            '-x', '50.0',
            '-y', '0.0',
            '-z', '0.2',
            '-Y', '0.0',
            '-robot_namespace', 'line_follower'
        ],
        output='screen'
    )
    
    # Controllers pour blue_line robot
    blue_robot_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/line_follower/controller_manager'],
        output='screen'
    )
    
    blue_robot_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/line_follower/controller_manager'],
        output='screen'
    )
    
    # Line Follower Node
    line_follower_node = Node(
        package='blue_line_follower',
        executable='line_follower_node',
        name='line_follower_node',
        namespace='line_follower',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # ====== RVIZ ======
    rviz_config = os.path.join(blue_line_pkg, 'config', 'robot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
    
    # ====== LAUNCH DESCRIPTION ======
    return LaunchDescription([
        # Environment
        set_gazebo_model_path,
        
        # Gazebo
        gazebo,
        
        # My Robot Controller (position originale)
        my_robot_state_publisher,
        spawn_my_robot,
        spawn_my_robot_diff_drive,
        spawn_my_robot_joint_broad,
        spawn_my_robot_arm,
        spawn_my_robot_gripper,
        my_robot_ekf,
        
        # Blue Line Follower (position décalée)
        blue_robot_state_publisher,
        spawn_blue_robot,
        blue_robot_joint_state_broadcaster,
        blue_robot_diff_drive_controller,
        line_follower_node,
        
        # Visualization
        rviz_node
    ])
