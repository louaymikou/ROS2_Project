#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_controller')
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    
    # Chemins des fichiers
    nav2_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    map_file = '/home/ikram/ROS2_Project/my_robot_map.yaml'
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Lancement de la simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'launch_sim.launch.py'])
            ])
        ),
        
        # Lancement de la localisation (AMCL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_bringup_share, 'launch', 'localization_launch.py'])
            ]),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
            }.items()
        ),
        
        # Lancement de Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
            }.items()
        ),
        
        # Remapping cmd_vel vers diff_cont
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            output='screen',
            arguments=['/cmd_vel', '/diff_cont/cmd_vel_unstamped'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
