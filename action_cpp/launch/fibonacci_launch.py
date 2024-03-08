from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_cpp',
            namespace='cpp',
            executable='action_server_diy',
            name='action_server_diy',
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='action_cpp',
            namespace='cpp',
            executable='action_client_diy',
            name='action_client_diy',
            output="screen",
            emulate_tty=True
        )
    ])