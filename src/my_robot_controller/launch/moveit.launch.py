import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    Launch MoveIt2 for arm control with Gazebo simulation
    """
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    srdf_file = os.path.join(pkg_share, 'config', 'moveit', 'my_robot.srdf')
    kinematics_yaml = os.path.join(pkg_share, 'config', 'moveit', 'kinematics.yaml')
    ompl_planning_yaml = os.path.join(pkg_share, 'config', 'moveit', 'ompl_planning.yaml')
    moveit_controllers_yaml = os.path.join(pkg_share, 'config', 'moveit', 'moveit_controllers.yaml')
    joint_limits_yaml = os.path.join(pkg_share, 'config', 'moveit', 'joint_limits.yaml')
    moveit_yaml = os.path.join(pkg_share, 'config', 'moveit', 'moveit.yaml')
    
    # Process robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Read SRDF
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Read kinematics yaml
    with open(kinematics_yaml, 'r') as f:
        kinematics_config = {'robot_description_kinematics': f.read()}
    
    # MoveIt planning pipeline configuration
    planning_pipelines_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    # Trajectory execution parameters
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Start Gazebo with world
    world_file_template = os.path.join(pkg_share, 'worlds', 'my_world.world')
    mesh_path = os.path.join(pkg_share, 'models', 'my_map.stl')
    
    with open(world_file_template, 'r') as f:
        world_content = f.read()
    world_content = world_content.replace('package://my_robot_controller/models/my_map.stl', f'file://{mesh_path}')
    
    import tempfile
    temp_dir = tempfile.gettempdir()
    world_file_path = os.path.join(temp_dir, 'my_world_resolved.world')
    with open(world_file_path, 'w') as f:
        f.write(world_content)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file_path}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )
    
    # Differential Drive Controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
        output='screen'
    )
    
    # Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )
    
    # Gripper Controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )
    
    # MoveIt Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {'use_sim_time': True},
        ],
    )
    
    # RViz with MoveIt
    rviz_config_file = os.path.join(pkg_share, 'config', 'moveit', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {'use_sim_time': True},
        ]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        diff_drive_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
    ])
