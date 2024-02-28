from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='service_py',
            namespace='python',
            executable='server_diy',
            name='server_diy'
        ),
        Node(
            package='service_py',
            namespace='python',
            executable='client_diy',
            name='client_diy'
        )
    ])