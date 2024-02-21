from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_py',
            namespace='python',
            executable='talker_diy',
            name='talker_diy'
        ),
        Node(
            package='pubsub_py',
            namespace='python',
            executable='listener_diy',
            name='listener_diy'
        )
    ])