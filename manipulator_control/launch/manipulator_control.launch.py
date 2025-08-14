from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'manipulator',
            default_value='ur5e_arm',
            description='MoveIt planning group for the manipulator'
        ),
        Node(
            package='manipulator_control',
            executable='manipulator_control_node',
            name='manipulator_control_node',
            output='screen',
            parameters=[
                {'manipulator': LaunchConfiguration('manipulator')},
                {'gripper_service_topic': 'gripper_service'},
                {'move_to_pose_action': 'move_to_pose'},
                {'task_sequence_action': 'execute_task_sequence'}
            ]
        )
    ])
