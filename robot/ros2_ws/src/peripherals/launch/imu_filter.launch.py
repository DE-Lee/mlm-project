from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):
    compiled = os.environ.get("need_compile", "False")
    use_global_tf = LaunchConfiguration("use_global_tf").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    frame_prefix = LaunchConfiguration("frame_prefix").perform(context)

    # DEBUG: Print values
    print(f"[IMU_FILTER DEBUG] compiled={compiled}, use_global_tf={use_global_tf}, namespace={namespace}, frame_prefix={frame_prefix}")

    if compiled == "True":
        calibration_package_path = get_package_share_directory("calibration")
    else:
        calibration_package_path = "/home/ubuntu/ros2_ws/src/calibration"
    
    calib_file_path = os.path.join(calibration_package_path, "config/imu_calib.yaml")
    if not os.path.exists(calib_file_path):
        raise FileNotFoundError(f"Calibration file not found: {calib_file_path}")

    # TF remapping
    if use_global_tf == "true":
        tf_remapping = ("tf", "/tf")
    else:
        tf_remapping = ("/tf", "tf")

    # IMU raw 토픽 경로 (namespace 적용)
    if namespace:
        imu_raw_topic = f"/{namespace}/ros_robot_controller/imu_raw"
    else:
        imu_raw_topic = "/ros_robot_controller/imu_raw"

    print(f"[IMU_FILTER DEBUG] imu_raw_topic={imu_raw_topic}, tf_remapping={tf_remapping}")

    imu_calib_node = Node(
        package="imu_calib",
        executable="apply_calib",
        name="imu_calib",
        namespace=namespace,
        output="screen",
        parameters=[{"calib_file": calib_file_path}],
        remappings=[
            ("raw", imu_raw_topic),
            ("corrected", "imu_corrected")
        ]
    )

    # Multi-robot: IMU frame_id에 frame_prefix 적용
    imu_frame_id = f"{frame_prefix}imu_link" if frame_prefix else "imu_link"

    imu_filter_node = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        name="imu_filter",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_mag": False,
                "do_bias_estimation": True,
                "do_adaptive_gain": True,
                "publish_debug_topics": True,
                "fixed_frame": imu_frame_id,  # Multi-robot: frame_prefix 적용
            }
        ],
        remappings=[
            tf_remapping,
            ("/imu/data_raw", "imu_corrected"),
            ("imu/data", "imu"),
            ("/imu/steady_state", "imu/steady_state"),  # Multi-robot: 절대→상대 경로 변환
            ("/imu/rpy/filtered", "imu/rpy/filtered"),  # Multi-robot: 절대→상대 경로 변환
        ]
    )

    return [
        LogInfo(msg=f"[IMU_FILTER] Starting with namespace={namespace}, use_global_tf={use_global_tf}"),
        TimerAction(
            period=5.0,
            actions=[imu_calib_node, imu_filter_node]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_global_tf", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("frame_prefix", default_value=""),  # Multi-robot frame_prefix
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == "__main__":
    from launch import LaunchService
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
