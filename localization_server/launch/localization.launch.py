import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='YAML map file to load for localization'
    )

    # Get config values
    map_file = LaunchConfiguration('map_file')

    # Paths
    nav2_yaml = os.path.join(
        get_package_share_directory('localization_server'),
        'config', 'amcl_config_sim.yaml'
    )

    map_file_path = PathJoinSubstitution([
        get_package_share_directory('map_server'), 'config', map_file
    ])

    rviz_config_file = os.path.join(
        get_package_share_directory('localization_server'),
        'rviz', 'local.rviz'
    )

    return LaunchDescription([
        declare_map_file,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': map_file_path}
            ]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        )
    ])
