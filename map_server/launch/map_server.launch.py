import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument named 'map_file'
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map YAML file in the config directory'
    )

    # Get package config path
    config_dir = os.path.join(
        get_package_share_directory('map_server'), 'config')
    rviz_config_file = os.path.join(
        get_package_share_directory('map_server'), 'rviz', 'map_display.rviz')
    # Define path to map file using LaunchConfiguration and PathJoinSubstitution
    map_file_path = PathJoinSubstitution([
        config_dir,
        LaunchConfiguration('map_file')
    ])

    return LaunchDescription([
        declare_map_file,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file_path
            }]
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
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        )
    ])
