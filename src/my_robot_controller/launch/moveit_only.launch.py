import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    Simplified MoveIt launch - assumes Gazebo and controllers are already running
    Use this after launching slam_mapping.launch.py or navigation.launch.py
    """
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    srdf_file = os.path.join(pkg_share, 'config', 'moveit', 'my_robot.srdf')
    kinematics_yaml = os.path.join(pkg_share, 'config', 'moveit', 'kinematics.yaml')
    moveit_controllers_yaml = os.path.join(pkg_share, 'config', 'moveit', 'moveit_controllers.yaml')
    joint_limits_yaml = os.path.join(pkg_share, 'config', 'moveit', 'joint_limits.yaml')
    
    # Process robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Read SRDF
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Read kinematics
    with open(kinematics_yaml, 'r') as f:
        kinematics_config = {'robot_description_kinematics': f.read()}
    
    # MoveIt configuration
    planning_pipelines_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
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
        name='rviz2_moveit',
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
        move_group_node,
        rviz_node,
    ])
