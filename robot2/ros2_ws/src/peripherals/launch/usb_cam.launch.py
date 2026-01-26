import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService

def generate_launch_description():
    compiled = os.environ.get("need_compile", "False")
    if compiled == "True":
        peripherals_package_path = get_package_share_directory("peripherals")
    else:
        peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"
    
    camera_node = Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            output="screen",
            name="usb_cam",
            parameters=[os.path.join(peripherals_package_path, "config", "usb_cam_param.yaml")],
            remappings=[
                ("~/image_raw", "image_raw"),
                ("~/camera_info", "camera_info"),
            ],
        )

    return LaunchDescription([camera_node])

if __name__ == "__main__":
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
