"""
Color Sender Launch File

이미지 압축 노드 실행용 런처.
네임스페이스를 지원하여 멀티 로봇 환경에서 사용 가능.

Usage:
    # 기본 실행 (네임스페이스 없음)
    ros2 launch camera_utils color_sender.launch.py

    # 네임스페이스 적용 (멀티 로봇)
    ros2 launch camera_utils color_sender.launch.py namespace:=/robot1
    ros2 launch camera_utils color_sender.launch.py namespace:=/robot2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('camera_utils')

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node (e.g., /robot1)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'params.yaml'),
        description='Path to the parameters file'
    )

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    # Color sender node
    color_sender_node = Node(
        package='camera_utils',
        executable='color_sender',
        name='color_sender_node',
        namespace=namespace,
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        color_sender_node,
    ])
