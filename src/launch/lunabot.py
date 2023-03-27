from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive',
            executable='drive',
            name='drive'
        ),
        Node(
            package='telemetry',
            executable='client',
            name='telemetry_client'
        ),
        Node(
            package='mining_arm',
            executable='mining_arm',
            name='mining_arm'
        )
    ])