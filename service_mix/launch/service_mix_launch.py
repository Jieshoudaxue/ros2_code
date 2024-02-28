from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='service_mix',
            namespace='mix',
            executable='server',
            name='server'
        ),
        Node(
            package='service_mix',
            namespace='mix',
            executable='client.py',
            name='client'
        )
    ])