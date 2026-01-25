"""
Namespace가 적용된 bringup launch 파일
TF는 global로 유지하여 multi-robot 환경에서도 동작

사용법:
ros2 launch bringup bringup_ns.launch.py robot_name:=robot1
"""
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ.get("need_compile", "False")  # 기본값 False로 변경
    if compiled == "True":
        bringup_package_path = get_package_share_directory("bringup")
    else:
        bringup_package_path = "/home/ubuntu/ros2_ws/src/bringup"

    robot_name = LaunchConfiguration("robot_name").perform(context)

    print(f"[BRINGUP_NS DEBUG] compiled={compiled}, robot_name={robot_name}, bringup_path={bringup_package_path}")

    # mlm_bringup.launch.py 포함 - launch_arguments로 전달
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_package_path, "launch/mlm_bringup.launch.py")),
        launch_arguments=[
            ("use_global_tf", "true"),
            ("robot_namespace", robot_name),
        ],
    )

    # namespace 적용
    if robot_name and robot_name != "/":
        bringup_group = GroupAction([
            PushRosNamespace(robot_name),
            bringup_launch
        ])
    else:
        bringup_group = bringup_launch

    return [
        LogInfo(msg=f"[BRINGUP_NS] robot_name={robot_name}, passing use_global_tf=true"),
        bringup_group
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="robot1"),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == "__main__":
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
