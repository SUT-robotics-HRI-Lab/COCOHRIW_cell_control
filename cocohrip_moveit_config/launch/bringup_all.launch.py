from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Start cocohrip_control/start_robot_control.launch.py
    start_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cocohrip_control"),
                "launch",
                "start_robot_control.launch.py"
            ])
        )
    )

    # Start cocohrip_moveit_config/move_group.launch.py
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cocohrip_moveit_config"),
                "launch",
                "move_group.launch.py"
            ])
        )
    )

    # Start cocohrip_moveit_config/moveit_rviz.launch.py
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cocohrip_moveit_config"),
                "launch",
                "moveit_rviz.launch.py"
            ])
        )
    )

    return LaunchDescription([
        start_robot_control,
        move_group,
        moveit_rviz,
    ])