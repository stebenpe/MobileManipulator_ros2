from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = get_package_share_directory('ros2_pyads')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            name='com_config',
            default_value='config/com_config.yaml',
            description='Path to ADS Configuration file'),
        DeclareLaunchArgument(
            name='plc_admin',
            default_value='config/plc_admin.yaml',
            description='Path to Admin Configuration file'),

        # EIP bridge simulator node (if simulation is true)
        GroupAction([
            Node(
                package='state_machine',
                executable='moma_beckhoff',
                output='screen',
                parameters=[
                    {'com_config': PathJoinSubstitution([config_file_path, LaunchConfiguration('com_config')])},
                    {'plc_admin': PathJoinSubstitution([config_file_path, LaunchConfiguration('plc_admin')])}
                ],
            )
        ]),
    ])
