"""
Full System Launch File

path_player (주행) + mlm_avoid (긴급 정지) 통합 실행.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for full system.

    path_player와 safety 시스템을 모두 실행한다.
    """
    # 패키지 경로
    bringup_dir = get_package_share_directory('mlm_avoid_bringup')
    path_player_dir = get_package_share_directory('path_player_pkg')

    # 파라미터 파일 경로
    avoid_params_file = os.path.join(bringup_dir, 'config', 'avoid_params.yaml')
    motion_params_file = os.path.join(path_player_dir, 'config', 'motion_params.yaml')
    path_player_params_file = os.path.join(path_player_dir, 'config', 'path_player_params.yaml')

    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=avoid_params_file,
        description='Path to avoid parameters file'
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

    # ========== mlm_avoid 노드들 ==========

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

    # ========== path_player 노드들 ==========

    # Path Player Server
    path_player_server = Node(
        package='path_player_pkg',
        executable='path_player_server',
        name='path_player_server',
        parameters=[path_player_params_file],
        output='screen',
        emulate_tty=True
    )

    # Motion Controller
    motion_controller = Node(
        package='path_player_pkg',
        executable='motion_controller',
        name='motion_controller',
        parameters=[motion_params_file],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        # Arguments
        params_file_arg,
        ttc_start_arg,
        ttc_end_arg,
        # Safety nodes (센서 동기화 + TTC 기반 긴급 정지)
        sync_node,
        safety_node,
        # Path player nodes (경로 추종 주행)
        path_player_server,
        motion_controller,
    ])
