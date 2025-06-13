from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator_control',
            executable='manipulator_control_node',
            name='manipulator_control_node',
            output='screen',
            parameters=[
                {'manipulator': 'ur_manipulator'},
                {'gripper_service_topic': 'gripper_service'},
                {'move_to_pose_action': 'move_to_pose'},
                {'task_sequence_action': 'execute_task_sequence'}
            ]
        )
    ])
