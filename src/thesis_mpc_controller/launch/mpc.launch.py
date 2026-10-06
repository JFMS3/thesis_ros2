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
    optitrack_package_name = 'thesis_optitrack_bridge'
    
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

    declare_reference_preview = DeclareLaunchArgument(
        'reference_preview_enabled',
        default_value='true',
        description='true if platform reference preview is enabled, false otherwise.'
    )
    declare_radius = DeclareLaunchArgument(
        'radius',
        default_value='0.8',
        description='Radius of circular path for platform controller'
    )
    declare_linspeed = DeclareLaunchArgument(
        'linspeed',
        default_value='0.1',
        description='Linear speed of the platform'
    )
    reference_preview_enabled = LaunchConfiguration('reference_preview_enabled')
    radius = LaunchConfiguration('radius')
    linspeed = LaunchConfiguration('linspeed')


    mpc_node = Node(
        package=mpc_package_name,
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=[mpc_yaml_path, {
            'reference_preview_enabled': reference_preview_enabled
        }],
        arguments=['--ros-args', '--log-level', 'info']
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

    platform_controller_node = Node(
        package=platform_package_name,
        executable='platform_controller',
        name='platform_controller',
        output='screen',
        parameters=[platform_yaml_path, {
            'RADIUS': radius,
            'LINSPEED': linspeed
        }]
    )
    
    platform_kalman_filter_node = Node(
        package=platform_package_name,
        executable='platform_kf',
        name='platform_kalman_filter',
        output='screen',
        parameters=[platform_yaml_path, {
            'log_dir': log_dir
        }]
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

    optitrack_bridge_node = Node(
        package=optitrack_package_name,
        executable='optitrack_bridge',
        name='optitrack_bridge',
        output='screen'
    )

    return LaunchDescription([
        declare_reference_preview,
        declare_radius,
        declare_linspeed,

        mpc_node,
        flight_control_node,
        platform_kalman_filter_node,
        platform_controller_node,
        quadcopter_kalman_filter_node,
        #optitrack_bridge_node
    ])
