from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_py',
            namespace='python',
            executable='fibonacci_server',
            name='fibonacci_server',
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='action_py',
            namespace='python',
            executable='fibonacci_client',
            name='fibonacci_client',
            output="screen",
            emulate_tty=True
        )
    ])