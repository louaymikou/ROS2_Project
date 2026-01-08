import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    Launch file for autonomous navigation with Nav2
    Use this AFTER creating a map with SLAM
    """
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Robot description with LIDAR
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # World file - with mesh path substitution
    world_file_template = os.path.join(pkg_share, 'worlds', 'my_world.world')
    mesh_path = os.path.join(pkg_share, 'models', 'my_map.stl')
    
    # Read world file and substitute mesh path
    with open(world_file_template, 'r') as f:
        world_content = f.read()
    world_content = world_content.replace('package://my_robot_controller/models/my_map.stl', f'file://{mesh_path}')
    
    # Create temporary world file with absolute paths
    import tempfile
    temp_dir = tempfile.gettempdir()
    world_file_path = os.path.join(temp_dir, 'my_world_resolved.world')
    with open(world_file_path, 'w') as f:
        f.write(world_content)
    
    # Map file path
    map_file = os.path.join(pkg_share, 'maps', 'my_robot_map.yaml')
    
    # Nav2 params
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
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
    
    # Map Server - DELAY 8 SECONDS
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_file}]
    )
    
    # Lifecycle manager for Map Server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    
    # AMCL (Localization) - DELAY 10 SECONDS
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Lifecycle manager for AMCL
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['amcl']}]
    )
    
    # Nav2 - DELAY 12 SECONDS
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_file
        }.items()
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
        
        # Start map server
        TimerAction(period=8.0, actions=[
            map_server,
            lifecycle_manager_map
        ]),
        
        # Start localization (AMCL)
        TimerAction(period=10.0, actions=[
            amcl,
            lifecycle_manager_localization
        ]),
        
        # Start Nav2
        TimerAction(period=12.0, actions=[nav2_bringup])
    ])
