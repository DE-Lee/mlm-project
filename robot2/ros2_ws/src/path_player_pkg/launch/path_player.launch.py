#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('path_player_pkg')
    
    # Path to config files
    motion_params_file = os.path.join(pkg_dir, 'config', 'motion_params.yaml')
    path_player_params_file = os.path.join(pkg_dir, 'config', 'path_player_params.yaml')
    
    # Motion Controller Node
    motion_controller_node = Node(
        package='path_player_pkg',
        executable='motion_controller',
        name='motion_controller',
        parameters=[motion_params_file],
        output='screen',
        emulate_tty=True
    )
    
    # Path Player Server Node
    path_player_server_node = Node(
        package='path_player_pkg',
        executable='path_player_server',
        name='path_player_server',
        parameters=[path_player_params_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        motion_controller_node,
        path_player_server_node
    ])


if __name__ == '__main__':
    generate_launch_description()
