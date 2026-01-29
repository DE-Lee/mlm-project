import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('perception_bridge'),
        'config',
        'params.yaml'
    )

    perception_bridge_node = Node(
        package='perception_bridge',
        executable='perception_bridge_node',
        name='perception_bridge_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        perception_bridge_node
    ])
