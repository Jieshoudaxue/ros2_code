from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_py',
            namespace='python',
            executable='talker',
            name='talker'
        ),
        Node(
            package='pubsub_py',
            namespace='python',
            executable='listener',
            name='listener'
        )
    ])