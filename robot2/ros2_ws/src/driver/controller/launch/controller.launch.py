import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, OpaqueFunction, LogInfo

def launch_setup(context):
    compiled = os.environ.get("need_compile", "False")
    namespace = LaunchConfiguration("namespace").perform(context)
    use_namespace = LaunchConfiguration("use_namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_odom = LaunchConfiguration("enable_odom")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    imu_frame = LaunchConfiguration("imu_frame")
    frame_prefix = LaunchConfiguration("frame_prefix")
    frame_prefix_str = LaunchConfiguration("frame_prefix").perform(context)  # 실제 문자열 값
    use_global_tf = LaunchConfiguration("use_global_tf").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    # DEBUG - Multi-robot frame_prefix 확인
    print(f"[CONTROLLER DEBUG] compiled={compiled}, use_global_tf={use_global_tf}, robot_namespace={robot_namespace}, frame_prefix={frame_prefix_str}")

    # Multi-robot: imu_frame에 frame_prefix 적용
    imu_frame_str = LaunchConfiguration("imu_frame").perform(context)
    imu_frame_with_prefix = f"{frame_prefix_str}{imu_frame_str}" if frame_prefix_str else imu_frame_str
    print(f"[CONTROLLER DEBUG] imu_frame_with_prefix={imu_frame_with_prefix}")

    if compiled == "True":
        peripherals_package_path = get_package_share_directory("peripherals")
        controller_package_path = get_package_share_directory("controller")
    else:
        peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"
        controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"

    # TF remapping - use_global_tf가 true면 global /tf 사용
    if use_global_tf == "true":
        tf_remappings = [("tf", "/tf"), ("tf_static", "/tf_static")]
    else:
        tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    odom_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(controller_package_path, "launch/odom_publisher.launch.py")
        ]),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "imu_frame": imu_frame_with_prefix,  # Multi-robot: frame_prefix 적용된 imu_frame
            "frame_prefix": frame_prefix,
            "base_frame": base_frame,
            "odom_frame": odom_frame,
            "use_global_tf": use_global_tf,
        }.items()
    )

    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(peripherals_package_path, "launch/imu_filter.launch.py")
        ]),
        launch_arguments={
            "use_global_tf": use_global_tf,
            "namespace": robot_namespace,
            "frame_prefix": frame_prefix_str,  # Multi-robot: IMU frame_id에 prefix 적용
        }.items()
    )

    # Multi-robot: frame_prefix를 사용하여 EKF frame 설정
    # ekf.yaml의 "namespace/" 플레이스홀더를 frame_prefix로 치환
    # 예: frame_prefix="robot2/" → namespace/odom → robot2/odom
    ekf_param = ReplaceString(
        source_file=os.path.join(controller_package_path, "config/ekf.yaml"),
        replacements={"namespace/": frame_prefix_str}
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_param, {"use_sim_time": use_sim_time}],
        remappings=tf_remappings + [
            ("odometry/filtered", "odom"),
            ("cmd_vel", "controller/cmd_vel")
        ],
        condition=IfCondition(enable_odom),
    )

    return [
        LogInfo(msg=f"[CONTROLLER] use_global_tf={use_global_tf}, robot_namespace={robot_namespace}"),
        imu_filter_launch,
        odom_publisher_launch,
        ekf_filter_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_namespace", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_odom", default_value="true"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("imu_frame", default_value="imu_link"),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("use_global_tf", default_value="false"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == "__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
