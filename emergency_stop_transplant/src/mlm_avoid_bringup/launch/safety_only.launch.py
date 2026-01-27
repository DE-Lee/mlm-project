"""
MLM Avoid Safety Only Launch File

Safety 노드만 실행하는 launch 파일.
TTC 기반 긴급 정지만 수행한다. (path_player 없이 독립 실행용)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description.

    launch description을 생성한다.
    """
    # 패키지 경로
    pkg_dir = get_package_share_directory('mlm_avoid_bringup')

    # 파라미터 파일 경로
    params_file = os.path.join(pkg_dir, 'config', 'avoid_params.yaml')

    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to parameters file'
    )

    ttc_start_arg = DeclareLaunchArgument(
        'ttc_start',
        default_value='2.0',
        description='TTC threshold to start intervention (seconds)'
    )

    ttc_end_arg = DeclareLaunchArgument(
        'ttc_end',
        default_value='3.0',
        description='TTC threshold to end intervention (seconds)'
    )

    # Sync Node
    sync_node = Node(
        package='mlm_avoid_sync',
        executable='sync_node',
        name='mlm_avoid_sync_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    # Safety Node
    safety_node = Node(
        package='mlm_avoid_safety',
        executable='safety_node',
        name='mlm_avoid_safety_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'ttc_threshold_start': LaunchConfiguration('ttc_start'),
                'ttc_threshold_end': LaunchConfiguration('ttc_end'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        ttc_start_arg,
        ttc_end_arg,
        sync_node,
        safety_node,
    ])
