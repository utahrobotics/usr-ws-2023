from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='local_gamepad_input',
            namespace='local_gamepad_input',
            executable='gamepad_node',
            name='local_gamepad_input'
        ),
        Node(
            package='telemetry',
            namespace='telemetry_server',
            executable='server',
            name='telementry_server'
        )
    ])