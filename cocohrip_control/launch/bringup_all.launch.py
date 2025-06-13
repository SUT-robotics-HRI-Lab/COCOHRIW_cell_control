from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments for the parent launch file
    from launch.actions import DeclareLaunchArgument

    declared_arguments = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.5"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
    ]

    start_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cocohrip_control"),
                "launch",
                "start_robot_control.launch.py"
            ])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cocohrip_moveit_config"),
                "launch",
                "move_group.launch.py"
            ])
        )
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("cocohrip_moveit_config"),
                "launch",
                "moveit_rviz.launch.py"
            ])
        )
    )

    gripper_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robotiq_hande_ros2_driver"),
                "launch",
                "gripper_bringup.launch.py"
            ])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [
            start_robot_control,
            move_group,
            moveit_rviz,
            gripper_control,
        ]
    )