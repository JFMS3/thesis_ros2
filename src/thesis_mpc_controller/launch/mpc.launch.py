import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'thesis_mpc_controller' 
    
    yaml_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'matrix_params.yaml'
    )

    mpc_node = Node(
        package=package_name,
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=[yaml_config_path]
    )

    flight_control_node = Node(
        package=package_name,
        executable='flight_control_node',
        name='flight_control_node',
        output='screen',
        parameters=[yaml_config_path]
    )

    return LaunchDescription([
        mpc_node,
        flight_control_node
    ])
