import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('sura_path_planning'), 'config')
    planner_config = os.path.join(config_dir, 'planner.yaml')
    controller_config = os.path.join(config_dir, 'graceful_controller.yaml')
    namespace = 'blueboat'

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[planner_config],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[controller_config],
            remappings=[('cmd_vel', '/blueboat/nav2/cmd_vel')],
        ),
        Node(
            package='sura_path_planning',
            executable='nav2_velocity_adapter',
            name='nav2_velocity_adapter',
            namespace=namespace,
            output='screen',
            parameters=[controller_config],
        ),
        Node(
            package='sura_path_planning',
            executable='simple_navigator',
            name='simple_navigator',
            namespace=namespace,
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            namespace=namespace,
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['planner_server']}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_controller',
            namespace=namespace,
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['controller_server']}],
        ),
    ])
