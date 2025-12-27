#!/usr/bin/env python3
"""
Hybrid Simulation Launch File
==============================
Lance la simulation complète avec le système hybride:
- Caméra Raspberry Pi V2 (SLAM visuel)
- 8 capteurs ultrasoniques HC-SR04 (évitement d'obstacles)
- IMU MPU6050 (fusion de capteurs)
- EKF sensor fusion
- Nav2 navigation stack
- SLAM Toolbox (basé sur ultrasons agrégés)

Usage:
  ros2 launch my_robot_controller hybrid_simulation.launch.py
  ros2 launch my_robot_controller hybrid_simulation.launch.py gui:=false  # Headless
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    use_nav = LaunchConfiguration('use_nav', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    gui = LaunchConfiguration('gui', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time')
    
    declare_use_slam = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Enable SLAM Toolbox')
    
    declare_use_nav = DeclareLaunchArgument(
        'use_nav', default_value='true',
        description='Enable Nav2 navigation stack')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 visualization')
    
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Enable Gazebo GUI (set false for headless)')
    
    # ==========================================================================
    # ROBOT DESCRIPTION - HYBRID VERSION
    # ==========================================================================
    xacro_file = os.path.join(pkg_share, 'description', 'robot_hybrid.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # ==========================================================================
    # CONFIG FILES
    # ==========================================================================
    world_file_path = os.path.join(pkg_share, 'worlds', 'my_world.world')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_hybrid_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_hybrid_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'hybrid_robot.rviz')
    
    # ==========================================================================
    # GAZEBO SIMULATION
    # ==========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'gui': gui
        }.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hybrid_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.3'
        ],
        output='screen'
    )
    
    # ==========================================================================
    # ROBOT STATE PUBLISHER
    # ==========================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # ==========================================================================
    # ROS2_CONTROL CONTROLLERS
    # ==========================================================================
    spawn_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )
    
    spawn_joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )
    
    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen"
    )
    
    spawn_gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen"
    )
    
    # ==========================================================================
    # ULTRASONIC AGGREGATOR
    # ==========================================================================
    # Combine 8 ultrasonic sensors into single LaserScan for Nav2
    ultrasonic_aggregator = Node(
        package='my_robot_controller',
        executable='ultrasonic_aggregator.py',
        name='ultrasonic_aggregator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 20.0,
            'range_min': 0.02,
            'range_max': 4.0,
            'interpolation_samples': 5
        }]
    )
    
    # ==========================================================================
    # EKF SENSOR FUSION (robot_localization)
    # ==========================================================================
    # Fuse wheel odometry + IMU for accurate pose estimation
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('odom0', '/diff_cont/odom'),
            ('imu0', '/imu/data')
        ]
    )
    
    # ==========================================================================
    # SLAM TOOLBOX
    # ==========================================================================
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_slam)
    )
    
    # ==========================================================================
    # NAV2 NAVIGATION STACK
    # ==========================================================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items(),
        condition=IfCondition(use_nav)
    )
    
    # ==========================================================================
    # RVIZ2 VISUALIZATION
    # ==========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # ==========================================================================
    # ACTION SERVERS (for pick-and-place missions)
    # ==========================================================================
    arm_action_server = Node(
        package='my_robot_controller',
        executable='arm_action_server.py',
        name='arm_action_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    pick_place_server = Node(
        package='my_robot_controller',
        executable='pick_place_action_server.py',
        name='pick_place_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ==========================================================================
    # DELAYED LAUNCHES (wait for Gazebo to initialize)
    # ==========================================================================
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            spawn_diff_drive,
            spawn_joint_broad,
            spawn_arm,
            spawn_gripper
        ]
    )
    
    delayed_localization = TimerAction(
        period=8.0,
        actions=[
            ultrasonic_aggregator,
            ekf_node
        ]
    )
    
    delayed_navigation = TimerAction(
        period=12.0,
        actions=[
            slam_toolbox,
            nav2_bringup
        ]
    )
    
    delayed_action_servers = TimerAction(
        period=15.0,
        actions=[
            arm_action_server,
            pick_place_server
        ]
    )
    
    # ==========================================================================
    # LAUNCH DESCRIPTION
    # ==========================================================================
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_use_slam,
        declare_use_nav,
        declare_use_rviz,
        declare_gui,
        
        # Core nodes (start immediately)
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
        
        # Delayed nodes
        delayed_controllers,
        delayed_localization,
        delayed_navigation,
        delayed_action_servers,
    ])
