import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_robot_controller'

    # 1. Configurer le robot (URDF)
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Le chemin vers votre nouveau fichier world
    world_file_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'my_world.world')

    # 2. Lancer Gazebo (serveur + client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        # On ajoute l'argument 'world'
        launch_arguments={'world': world_file_path}.items()
    )

    # 3. Spawner le robot dans Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-x', '0',
                                   '-y', '0',
                                   '-z', '0.3'],  # Divisé par 2 : 0.6 -> 0.3
                        output='screen')

    # 4. Robot State Publisher (Publie les transformations)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. Spawner les contrôleurs (DIFF DRIVE)
    spawn_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )

    # 6. Spawner les contrôleurs (JOINT STATE)
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

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        spawn_diff_drive,
        spawn_joint_broad,
        spawn_arm
    ])