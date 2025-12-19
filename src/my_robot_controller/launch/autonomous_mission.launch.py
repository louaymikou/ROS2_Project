import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    map_file = LaunchConfiguration('map_file', default='')
    
    # Robot description with LIDAR
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # World file
    world_file_path = os.path.join(pkg_share, 'worlds', 'my_world.world')
    
    # Config files
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # ==================== GAZEBO ====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file_path, 'verbose': 'false'}.items()
    )
    
    # Spawn robot
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
    
    # ==================== ROBOT STATE PUBLISHER ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # ==================== ROS2_CONTROL CONTROLLERS ====================
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
    
    # ==================== SLAM TOOLBOX ====================
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ==================== NAV2 ====================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )
    
    # ==================== ACTION SERVERS ====================
    # Delay action servers to ensure Nav2 is ready
    pick_place_server = TimerAction(
        period=8.0,  # Wait 8 seconds for Nav2 to initialize
        actions=[
            Node(
                package=pkg_name,
                executable='pick_place_action_server.py',
                name='pick_place_action_server',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    arm_action_server = TimerAction(
        period=5.0,
        actions=[
            Node(
                package=pkg_name,
                executable='arm_action_server.py',
                name='arm_action_server',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('map_file', default_value=''),
        
        # Gazebo
        gazebo,
        robot_state_publisher,
        spawn_entity,
        
        # Controllers
        spawn_diff_drive,
        spawn_joint_broad,
        spawn_arm,
        spawn_gripper,
        
        # SLAM
        slam_toolbox,
        
        # Nav2
        TimerAction(
            period=3.0,  # Wait for SLAM to initialize
            actions=[nav2_bringup]
        ),
        
        # Action Servers
        arm_action_server,
        pick_place_server,
    ])
