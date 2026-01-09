#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Packages
    pkg_share = FindPackageShare('my_robot_controller').find('my_robot_controller')
    nav2_bringup_share = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Fichiers
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = '/home/ikram/ROS2_Project/my_robot_map.yaml'

    # 1️⃣ Gazebo + robot
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_sim.launch.py')
        )
    )

    # 2️⃣ Nav2 (localization + navigation) avec AUTOSTART
    launch_nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true'
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # 3️⃣ cmd_vel relay
    cmd_vel_relay = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='topic_tools',
                executable='relay',
                name='cmd_vel_relay',
                output='screen',
                arguments=['/cmd_vel_nav', '/diff_cont/cmd_vel_unstamped'],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 4️⃣ Pose initiale
    set_initial_pose = TimerAction(
        period=28.0,
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        launch_sim,
        launch_nav2,
        cmd_vel_relay,
        set_initial_pose,
    ])
