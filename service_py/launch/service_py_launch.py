from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='service_py',
            namespace='python',
            executable='server',
            name='server'
        ),
        Node(
            package='service_py',
            namespace='python',
            executable='client',
            name='client'
        )
    ])