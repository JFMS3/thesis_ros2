import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
from pathlib import Path

def generate_launch_description():
    mpc_package_name = 'thesis_mpc_controller'
    platform_package_name = 'thesis_platform_controller'
    
    run_name = datetime.now().strftime("mpc_%m-%d_%H-%M")
    log_dir = str(Path.home() / "thesis_logs" / run_name)
    Path(log_dir).mkdir(parents=True, exist_ok=True)
    
    mpc_yaml_path = os.path.join(
        get_package_share_directory(mpc_package_name),
        'config',
        'matrix_params.yaml'
    )

    platform_yaml_path = os.path.join(
        get_package_share_directory(platform_package_name),
        'config',
        'matrix_params.yaml'
    )

    mpc_node = Node(
        package=mpc_package_name,
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=[mpc_yaml_path]
    )

    flight_control_node = Node(
        package=mpc_package_name,
        executable='flight_control_node',
        name='flight_control_node',
        output='screen',
        parameters=[mpc_yaml_path, {
            'log_dir': log_dir
        }]
    )

    platform_kalman_filter_node = Node(
        package=platform_package_name,
        executable='platform_kf',
        name='platform_kalman_filter',
        output='screen',
        parameters=[platform_yaml_path]
    )

    quadcopter_kalman_filter_node = Node(
        package=mpc_package_name,
        executable='quadcopter_kf',
        name='quadcopter_kf',
        output='screen',
        parameters=[mpc_yaml_path, {
            'log_dir': log_dir
        }]
    )

    return LaunchDescription([
        mpc_node,
        flight_control_node,
        platform_kalman_filter_node,
        quadcopter_kalman_filter_node
    ])
