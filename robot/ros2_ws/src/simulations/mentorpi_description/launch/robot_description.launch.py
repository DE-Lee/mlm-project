import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription, LaunchService
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_global_tf = LaunchConfiguration('use_global_tf').perform(context)

    print(f"[ROBOT_DESCRIPTION DEBUG] compiled={compiled}, use_global_tf={use_global_tf}, frame_prefix={frame_prefix}")

    if compiled == 'True':
        mentorpi_description_package_path = get_package_share_directory('mentorpi_description')
    else:
        mentorpi_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/mentorpi_description'
    urdf_path = os.path.join(mentorpi_description_package_path, 'urdf/mentorpi.xacro')
    rviz_config_file = os.path.join(mentorpi_description_package_path, 'rviz/view.rviz')

    robot_description = Command(['xacro ', urdf_path])

    if use_global_tf == "true":
        tf_remappings = [('tf', '/tf'), ('tf_static', '/tf_static')]
    else:
        tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    print(f"[ROBOT_DESCRIPTION DEBUG] tf_remappings={tf_remappings}")

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['/controller_manager/joint_states'], 'rate': 20.0}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
        remappings=[('/joint_states', 'joint_controller')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'frame_prefix': frame_prefix, 'use_sim_time': use_sim_time}],
        arguments=[urdf_path],
        remappings=tf_remappings
    )

    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mentorpi_description_package_path, 'launch', 'rviz.launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={'namespace': namespace, 'use_namespace': use_namespace, 'rviz_config': rviz_config_file}.items())

    return [
        joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_launch,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_namespace', default_value='false'),
        DeclareLaunchArgument('use_global_tf', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
