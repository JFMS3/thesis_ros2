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
    mpc_yaml_path = os.path.join(
        get_package_share_directory(mpc_package_name),
        'config',
        'matrix_params.yaml'
    )

    hover_thrust_arg = DeclareLaunchArgument(
        'hover_thrust',
        default_value='39200',
        description='Hover thrust for crazyflie'
    )

    run_name = datetime.now().strftime(
        "velocity_kf_estimation_%m-%d_%H-%M"
    )
    log_dir = str(Path.home() / "thesis_logs" / run_name)
    Path(log_dir).mkdir(parents=True, exist_ok=True)


    kf_node = Node(
        package=mpc_package_name,
        executable='quadcopter_kf',
        name='quadcopter_kf',
        output='screen',
        parameters=[mpc_yaml_path, {
            'log_dir': log_dir
        }]
    )

    simple_hover_node = Node(
        package='thesis_optitrack_bridge',
        executable='simple_hover_node',
        name='simple_hover_node',
        output='screen',
        parameters=[{
            'HOVER_THRUST': LaunchConfiguration('hover_thrust'),
            'log_dir': log_dir
        }]
    )


    return LaunchDescription([
        hover_thrust_arg,
        kf_node,
        simple_hover_node
    ])
