import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction

def launch_setup(context):
    compiled = os.environ.get("need_compile", "False")
    namespace = LaunchConfiguration("namespace").perform(context)
    use_namespace = LaunchConfiguration("use_namespace").perform(context)
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    imu_frame = LaunchConfiguration("imu_frame")
    frame_prefix = LaunchConfiguration("frame_prefix")
    use_global_tf = LaunchConfiguration("use_global_tf").perform(context)

    if compiled == "True":
        rosmentor_description_package_path = get_package_share_directory("mentorpi_description")
        robot_controller_package_path = get_package_share_directory("ros_robot_controller")
        controller_package_path = get_package_share_directory("controller")
    else:
        rosmentor_description_package_path = "/home/ubuntu/ros2_ws/src/simulations/mentorpi_description"
        robot_controller_package_path = "/home/ubuntu/ros2_ws/src/driver/ros_robot_controller"
        controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(rosmentor_description_package_path, "launch/robot_description.launch.py")
        ]),
        launch_arguments={
            "frame_prefix": frame_prefix,
            "use_gui": "false",
            "use_rviz": "false",
            "use_sim_time": "false",
            "use_namespace": use_namespace,
            "namespace": namespace,
            "use_global_tf": use_global_tf,
        }.items()
    )

    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_controller_package_path, "launch/ros_robot_controller.launch.py")
        ]),
        launch_arguments={
            "imu_frame": imu_frame,
        }.items()
    )

    odom_publisher_node = Node(
        package="controller",
        executable="odom_publisher",
        name="odom_publisher",
        output="screen",
        parameters=[os.path.join(controller_package_path, "config/calibrate_params.yaml"), {
            "base_frame_id": base_frame, 
            "odom_frame_id": odom_frame,
            "pub_odom_topic": True,
            }],  
    )

    return [
        robot_description_launch,
        robot_controller_launch,
        odom_publisher_node
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("imu_frame", default_value="imu_link"),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("use_global_tf", default_value="false"),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == "__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
