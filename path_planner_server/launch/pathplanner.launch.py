from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')
    controller_yaml = os.path.join(config_dir, 'controller_sim.yaml' if use_sim_time else 'controller_real.yaml')
    bt_navigator_yaml = os.path.join(config_dir, 'bt_navigator_sim.yaml' if use_sim_time else 'bt_navigator_real.yaml')
    planner_yaml = os.path.join(config_dir, 'planner_sim.yaml' if use_sim_time else 'planner_real.yaml')
    recovery_yaml = os.path.join(config_dir, 'recoveries_sim.yaml' if use_sim_time else 'recoveries_real.yaml')
    rviz_config_file = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')
    # cmd_vel_remap = ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
    cmd_vel_remap = ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped' if use_sim_time else '/cmd_vel')

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[cmd_vel_remap]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time}]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_pathplanner',
                    output='screen',
                    parameters=[
                        {'autostart': True},
                        {'node_names': [
                            'planner_server',
                            'controller_server',
                            'recoveries_server',
                            'bt_navigator'
                        ]},
                        {'use_sim_time': use_sim_time}
                    ]
                )
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        OpaqueFunction(function=launch_setup)
    ])
