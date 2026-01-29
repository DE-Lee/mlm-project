import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    """Multi-robot 지원을 위한 frame_prefix 처리"""
    compiled = os.environ.get("need_compile", "False")
    if compiled == "True":
        peripherals_package_path = get_package_share_directory("peripherals")
    else:
        peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"

    # frame_prefix 값 가져오기 (예: "robot1/")
    frame_prefix = LaunchConfiguration("frame_prefix").perform(context)

    # Multi-robot: frame_id에 prefix 적용
    camera_frame_id = f"{frame_prefix}camera" if frame_prefix else "camera"

    camera_node = Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            output="screen",
            name="usb_cam",
            parameters=[
                os.path.join(peripherals_package_path, "config", "usb_cam_param.yaml"),
                {"frame_id": camera_frame_id},  # Multi-robot: frame_prefix 적용
            ],
        )

    return [camera_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("frame_prefix", default_value=""),  # Multi-robot frame_prefix
        OpaqueFunction(function=launch_setup),
    ])

if __name__ == "__main__":
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
