import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    Launch file for building map with SLAM
    Use this for first-time mapping, then save the map
    """
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Robot description with LIDAR
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # World file
    world_file_path = os.path.join(pkg_share, 'worlds', 'my_world.world')
    
    # SLAM params
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file_path}.items()
    )
    
    # Robot state publisher - START IMMEDIATELY
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Spawn robot - DELAY 3 SECONDS
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-x', '0', '-y', '0', '-z', '0.25'],
        output='screen'
    )
    
    # Controllers - DELAY 5 SECONDS
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
    
    # SLAM Toolbox - DELAY 8 SECONDS
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}]
    )
        
    return LaunchDescription([
            # Start immediately
            gazebo,
            robot_state_publisher,
            
            # Wait for Gazebo
            TimerAction(period=3.0, actions=[spawn_entity]),
            
            # Spawn controllers
            TimerAction(period=5.0, actions=[
                spawn_diff_drive,
                spawn_joint_broad,
                spawn_arm,
                spawn_gripper
            ]),
            
            # Start SLAM
            TimerAction(period=7.0, actions=[slam_toolbox])
        ])