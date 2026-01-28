import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    # CRITICAL: ROS_DOMAIN_ID 검증 (멀티로봇 통신 필수)
    domain_id = os.environ.get('ROS_DOMAIN_ID', None)
    if domain_id != '3':
        raise RuntimeError(
            f"❌ ERROR: ROS_DOMAIN_ID must be 3 for multi-robot system!\n"
            f"   Current: {domain_id if domain_id else 'NOT SET'}\n"
            f"   Fix: export ROS_DOMAIN_ID=3"
        )

    compiled = os.environ.get("need_compile", "False")
    use_global_tf = LaunchConfiguration("use_global_tf").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    frame_prefix = LaunchConfiguration("frame_prefix").perform(context)

    # Multi-robot: LiDAR frame_id에도 prefix 적용
    # 예: frame_prefix="robot2/" → lidar_frame="robot2/lidar_frame"
    lidar_frame = f"{frame_prefix}lidar_frame" if frame_prefix else "lidar_frame"

    print(f"[MLM_BRINGUP DEBUG] compiled={compiled}, use_global_tf={use_global_tf}, robot_namespace={robot_namespace}, frame_prefix={frame_prefix}")

    if compiled == "True":
        controller_package_path = get_package_share_directory("controller")
        peripherals_package_path = get_package_share_directory("peripherals")
        avoid_package_path = get_package_share_directory("mlm_avoid_bringup")
    else:
        controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"
        peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"
        avoid_package_path = "/home/ubuntu/ros2_ws/src/mlm_avoid_bringup"

    startup_check_node = Node(
        package="bringup",
        executable="startup_check",
        output="screen",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, "launch/controller.launch.py")),
        launch_arguments={
            "use_global_tf": use_global_tf,
            "robot_namespace": robot_namespace,
            "frame_prefix": frame_prefix,  # Multi-robot frame_prefix
        }.items(),
    )

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, "launch/usb_cam.launch.py")),
    )

    # Multi-robot: LiDAR frame_id에 prefix 적용
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, "launch/lidar.launch.py")),
        launch_arguments={
            "lidar_frame": lidar_frame,  # 예: robot2/lidar_frame
        }.items(),
    )

    # rosbridge_websocket_launch = ExecuteProcess(
    #         cmd=["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
    #         output="screen"
    #     )

    init_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_package_path, "launch/init_pose.launch.py")),
        launch_arguments={
            "namespace": "",
            "use_namespace": "false",
            "action_name": "init",
        }.items(),
    )

    # Emergency Stop 런치 (Nav2 자율주행용 TTC 기반 안전 시스템)
    # namespace는 PushRosNamespace로 상위에서 적용됨
    avoid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(avoid_package_path, "launch/avoid.launch.py")),
    )

    return [
            LogInfo(msg=f"[MLM_BRINGUP] use_global_tf={use_global_tf}, robot_namespace={robot_namespace}"),
            startup_check_node,
            controller_launch,
            usb_cam_launch,
            lidar_launch,
            # rosbridge_websocket_launch,  # PC에서 실행
            init_pose_launch,
            avoid_launch,  # Emergency Stop (Nav2 자율주행용)
            ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_global_tf", default_value="false"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("frame_prefix", default_value=""),  # Multi-robot frame_prefix
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == "__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
