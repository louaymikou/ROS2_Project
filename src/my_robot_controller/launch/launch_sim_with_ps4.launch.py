import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_robot_controller'

    # Charger le fichier URDF/Xacro
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Fichier monde Gazebo
    world_file_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'my_world.world')

    # Lancer Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # Spawn du robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.3'],
        output='screen'
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Contrôleurs
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

    # Nœud Joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    # Contrôleur PS4 - use package lib path where scripts are installed
    pkg_prefix = get_package_prefix(pkg_name)
    ps4_controller_path = os.path.join(pkg_prefix, 'lib', pkg_name, 'ps4_controller.py')
    ps4_controller_node = ExecuteProcess(
        cmd=['python3', ps4_controller_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        spawn_diff_drive,
        spawn_joint_broad,
        spawn_arm,
        spawn_gripper,
        joy_node,
        ps4_controller_node
    ])
