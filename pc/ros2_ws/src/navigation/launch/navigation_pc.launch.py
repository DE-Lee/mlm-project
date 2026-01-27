"""
PC에서 Navigation 스택만 실행하는 launch 파일 (Standalone 노드 방식)
로봇에서는 bringup.launch.py로 센서 드라이버만 실행

사용법:
ros2 launch navigation navigation_pc.launch.py map:=/path/to/map.yaml robot_name:=robot1
"""
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from nav2_common.launch import RewrittenYaml

def launch_setup(context):
    # CRITICAL: ROS_DOMAIN_ID 검증
    domain_id = os.environ.get('ROS_DOMAIN_ID', None)
    if domain_id != '3':
        raise RuntimeError(
            f"❌ ERROR: ROS_DOMAIN_ID must be 3 for multi-robot system!\n"
            f"   Current: {domain_id if domain_id else 'NOT SET'}\n"
            f"   Fix: export ROS_DOMAIN_ID=3"
        )

    # 항상 패키지 경로 사용
    slam_package_path = get_package_share_directory('slam')
    navigation_package_path = get_package_share_directory('navigation')

    map_name = LaunchConfiguration('map').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_teb = LaunchConfiguration('use_teb').perform(context)

    print(f"[NAV_PC DEBUG] map={map_name}, robot_name={robot_name}, use_teb={use_teb}")

    # map 파일 경로 처리 - 절대 경로가 아니면 maps 폴더에서 찾음
    if os.path.isabs(map_name):
        map_yaml_file = map_name
    elif map_name.endswith('.yaml'):
        map_yaml_file = os.path.join(slam_package_path, 'maps', map_name)
    else:
        map_yaml_file = os.path.join(slam_package_path, 'maps', map_name + '.yaml')

    print(f"[NAV_PC DEBUG] map_yaml_file={map_yaml_file}")

    params_file_path = os.path.join(navigation_package_path, 'config', 'nav2_params_ackermann.yaml')

    # RewrittenYaml을 사용하여 네임스페이스에 맞게 파라미터 재작성
    param_substitutions = {
        'use_sim_time': 'False',
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file_path,
            root_key=robot_name,  # 네임스페이스를 root_key로 사용
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # TF remapping - global /tf 사용 (로봇이 global /tf로 발행)
    remappings = [('tf', '/tf'), ('tf_static', '/tf_static')]

    # Costmap scan topic remapping - costmap 노드가 올바른 scan 토픽 구독하도록
    scan_topic = f'/{robot_name}/scan_raw' if robot_name else '/scan_raw'
    costmap_remappings = remappings + [('scan_raw', scan_topic)]

    # Lifecycle nodes
    lifecycle_nodes = ['map_server', 'amcl']
    navigation_lifecycle_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

    # namespace 설정
    ns = robot_name if robot_name and robot_name != '/' else ''

    # Navigation 노드들 (Standalone 방식 - namespace 직접 지정)
    nodes = [
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=ns,
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml_file, 'use_sim_time': False}],
            remappings=remappings,
        ),
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            output='screen',
            parameters=[configured_params, {
                'use_sim_time': False,
                'scan_topic': 'scan_raw',
                'set_initial_pose': True,
                'initial_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            }],
            remappings=remappings,
        ),
        # Controller Server (local_costmap 포함)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=ns,
            output='screen',
            parameters=[configured_params, {
                'use_sim_time': False,
                'controller_plugins': ['FollowPath'],
                'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            }],
            remappings=costmap_remappings + [('cmd_vel', 'cmd_vel_nav')],  # Emergency Stop: safety_node 경유
        ),
        # Planner Server (global_costmap 포함)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=ns,
            output='screen',
            parameters=[configured_params, {'use_sim_time': False}],
            remappings=costmap_remappings,
        ),
        # Behavior Server (cmd_vel → cmd_vel_nav: Emergency Stop 경유)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=ns,
            output='screen',
            parameters=[configured_params, {'use_sim_time': False}],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=ns,
            output='screen',
            parameters=[configured_params, {'use_sim_time': False}],
            remappings=remappings,
        ),
        # Lifecycle Manager for Localization
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=ns,
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': lifecycle_nodes,
                'use_sim_time': False,
                'bond_timeout': 30.0,  # 10.0 → 30.0 (로봇 부팅 대기 시간 확보)
            }],
        ),
        # Lifecycle Manager for Navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=ns,
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': navigation_lifecycle_nodes,
                'use_sim_time': False,
                'bond_timeout': 30.0,  # 10.0 → 30.0 (로봇 부팅 대기 시간 확보)
            }],
        ),
    ]

    bringup_group = GroupAction(nodes)

    return [bringup_group]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='map'),
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('use_teb', default_value='true'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
