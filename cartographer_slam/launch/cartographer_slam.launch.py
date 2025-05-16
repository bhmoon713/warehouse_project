import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config')

    rviz_config_file = os.path.join(
        get_package_share_directory('cartographer_slam'), 'rviz', 'mapping.rviz')

    def launch_setup(context, *args, **kwargs):
        sim_time = context.launch_configurations['use_sim_time'].lower() == 'true'
        config_file = 'cartographer.lua' if sim_time else 'cartographer_real.lua'

        return [
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': sim_time}],
                arguments=[
                    '-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', config_file
                ]
            ),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='occupancy_grid_node',
                output='screen',
                parameters=[{'use_sim_time': sim_time}],
                arguments=[
                    '-resolution', '0.05',
                    '-publish_period_sec', '1.0'
                ]
            ),
        ]

    return LaunchDescription([
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup)
    ])
