import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'thesis_platform_controller' 
    
    yaml_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'matrix_params.yaml'
    )

    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='0.8',
        description='Radius of circle path'
    )

    linspeed_arg = DeclareLaunchArgument(
        'linspeed',
        default_value='0.1',
        description='Linear speed of platform'
    )

    platform_controller_node = Node(
        package=package_name,
        executable='platform_controller',
        name='platform_controller',
        output='screen',
        parameters=[{
            'RADIUS': LaunchConfiguration('radius'),
            'LINSPEED': LaunchConfiguration('linspeed')
        }]
    )

    kalman_filter_node = Node(
        package=package_name,
        executable='platform_kf',
        name='platform_kalman_filter',
        output='screen',
        parameters=[yaml_config_path]
    )

    return LaunchDescription([
        radius_arg,
        linspeed_arg,
        platform_controller_node,
        kalman_filter_node
    ])
