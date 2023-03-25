from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='local_gamepad',
            # namespace='local_gamepad',
            executable='local_gamepad',
            name='local_gamepad'
        ),
        Node(
            package='telemetry',
            # namespace='telemetry_server',
            executable='server',
            name='telemetry_server'
        )
    ])