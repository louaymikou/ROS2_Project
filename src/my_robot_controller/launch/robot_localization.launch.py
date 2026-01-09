import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_controller'
    
    # Path to EKF parameters
    ekf_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf_params.yaml'
    )
    
    # EKF node for sensor fusion (wheel odom + IMU)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', 'odometry/local')
        ]
    )
    
    return LaunchDescription([
        ekf_node
    ])
