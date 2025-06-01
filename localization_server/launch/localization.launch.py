import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    map_file_value = LaunchConfiguration('map_file').perform(context)

    # Determine flags from map filename
    use_real = map_file_value.endswith('_real.yaml')
    use_sim_time = not use_real

    # Paths
    pkg_localization = get_package_share_directory('localization_server')
    pkg_map_server = get_package_share_directory('map_server')

    amcl_sim_config = os.path.join(pkg_localization, 'config', 'amcl_config_sim.yaml')
    amcl_real_config = os.path.join(pkg_localization, 'config', 'amcl_config_real.yaml')
    map_file_path = PathJoinSubstitution([pkg_map_server, 'config', map_file_value])
    rviz_config_file = os.path.join(pkg_localization, 'rviz', 'local.rviz')

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file_path}
            ]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_real_config if use_real else amcl_sim_config]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        )
    ]


def generate_launch_description():
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='YAML map file to load for localization'
    )

    return LaunchDescription([
        declare_map_file,
        OpaqueFunction(function=launch_setup)
    ])
