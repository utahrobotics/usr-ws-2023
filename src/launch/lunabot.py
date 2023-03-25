from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive',
            namespace='drive',
            executable='drive',
            name='drive'
        ),
        Node(
            package='telemetry',
            namespace='telemetry_client',
            executable='client',
            name='telementry_client'
        ),
        Node(
            package='mining_arm',
            namespace='mining_arm',
            executable='mining_arm',
            name='mining_arm'
        )
    ])