#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Packages
    pkg_share = FindPackageShare('my_robot_controller').find('my_robot_controller')
    nav2_bringup_share = FindPackageShare('nav2_bringup').find('nav2_bringup')
    
    # Fichiers de configuration
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = '/home/ikram/ROS2_Project/my_robot_map.yaml'
    rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. LANCEMENT DE GAZEBO + ROBOT
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_sim.launch.py')
        )
    )
    
    # 2. LANCEMENT DE NAV2 (MAP + LOCALIZATION + NAVIGATION) avec délai
    launch_nav2 = TimerAction(
        period=8.0,  # Attendre 8 secondes pour que Gazebo démarre
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                }.items()
            )
        ]
    )
    
    # 3. CMD_VEL RELAY avec délai
    cmd_vel_relay = TimerAction(
        period=10.0,  # Attendre 10 secondes
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='cmd_vel_relay',
                output='screen',
                arguments=['/cmd_vel', '/diff_cont/cmd_vel_unstamped'],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 4. PUBLICATION DE LA POSE INITIALE avec délai
    set_initial_pose = TimerAction(
        period=12.0,  # Attendre 12 secondes
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '/initialpose',
                    'geometry_msgs/PoseWithCovarianceStamped',
                    '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}',
                    '--once'
                ],
                output='screen'
            )
        ]
    )
    
    # 5. LANCEMENT DE RVIZ2 avec délai
    launch_rviz = TimerAction(
        period=13.0,  # Attendre 13 secondes
        actions=[
            ExecuteProcess(
                cmd=['rviz2', '-d', rviz_config],
                output='screen',
                shell=False,
                emulate_tty=True,
                env={'GTK_PATH': ''}  # Unset GTK_PATH
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Lancer dans l'ordre avec délais
        launch_sim,        # T+0s: Gazebo
        launch_nav2,       # T+8s: Nav2
        cmd_vel_relay,     # T+10s: Relay
        set_initial_pose,  # T+12s: Initial pose
        launch_rviz,       # T+13s: RViz
    ])
