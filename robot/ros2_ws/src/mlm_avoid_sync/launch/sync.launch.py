"""
MLM Avoid Sync Node Launch File

센서 동기화 노드를 실행하는 launch 파일.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""

    # Launch arguments
    buffer_size_arg = DeclareLaunchArgument(
        'buffer_size',
        default_value='30',
        description='Buffer size for each sensor'
    )

    slop_arg = DeclareLaunchArgument(
        'slop',
        default_value='0.1',
        description='Max time difference for synchronization (seconds)'
    )

    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='0.5',
        description='Timeout for cmd_vel sensors before using zero (seconds)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Image topic name'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan_raw',
        description='LiDAR scan topic name'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic name'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='IMU topic name'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Nav2 cmd_vel topic name'
    )

    controller_cmd_vel_topic_arg = DeclareLaunchArgument(
        'controller_cmd_vel_topic',
        default_value='/controller/cmd_vel',
        description='Controller cmd_vel topic name (emergency avoidance)'
    )

    # Sync Node
    sync_node = Node(
        package='mlm_avoid_sync',
        executable='sync_node',
        name='mlm_avoid_sync_node',
        parameters=[{
            'buffer_size': LaunchConfiguration('buffer_size'),
            'slop': LaunchConfiguration('slop'),
            'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            'image_topic': LaunchConfiguration('image_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'controller_cmd_vel_topic': LaunchConfiguration('controller_cmd_vel_topic'),
        }],
        output='screen',
    )

    return LaunchDescription([
        buffer_size_arg,
        slop_arg,
        cmd_vel_timeout_arg,
        image_topic_arg,
        scan_topic_arg,
        odom_topic_arg,
        imu_topic_arg,
        cmd_vel_topic_arg,
        controller_cmd_vel_topic_arg,
        sync_node,
    ])
