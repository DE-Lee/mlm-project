"""
MLM Avoid System Launch File

긴급 정지 시스템의 노드를 실행하는 launch 파일.
sync_node와 safety_node를 함께 실행한다.

Multi-robot 환경에서 namespace를 지원한다.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """
    Setup launch - namespace는 PushRosNamespace로 상위에서 적용됨.

    YAML 와일드카드 키(/**/)로 파라미터 로딩 지원.
    Node에 명시적 namespace 설정하면 PushRosNamespace와 중복되므로 제거.
    """
    # 패키지 경로 (표준 ROS 2 방식)
    pkg_dir = get_package_share_directory('mlm_avoid_bringup')

    # 파라미터 파일 경로
    params_file = os.path.join(pkg_dir, 'config', 'avoid_params.yaml')

    # Get launch configurations
    ttc_start = LaunchConfiguration('ttc_start')
    ttc_end = LaunchConfiguration('ttc_end')

    # Sync Node (namespace는 PushRosNamespace로 자동 적용)
    sync_node = Node(
        package='mlm_avoid_sync',
        executable='sync_node',
        name='mlm_avoid_sync_node',
        parameters=[params_file],
        output='screen',
    )

    # Safety Node (namespace는 PushRosNamespace로 자동 적용)
    safety_node = Node(
        package='mlm_avoid_safety',
        executable='safety_node',
        name='mlm_avoid_safety_node',
        parameters=[
            params_file,
            {
                'ttc_threshold_start': ttc_start,
                'ttc_threshold_end': ttc_end,
            }
        ],
        output='screen',
    )

    return [sync_node, safety_node]


def generate_launch_description():
    """
    Generate launch description.

    launch description을 생성한다.
    namespace는 PushRosNamespace로 상위에서 적용되므로 별도 argument 불필요.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'ttc_start',
            default_value='2.0',
            description='TTC threshold to start intervention (seconds)'
        ),
        DeclareLaunchArgument(
            'ttc_end',
            default_value='3.0',
            description='TTC threshold to end intervention (seconds)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
