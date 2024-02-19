from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_cpp',
            namespace='cpp',
            executable='talker_diy',
            name='talker_diy'
        ),
        Node(
            package='pubsub_cpp',
            namespace='cpp',
            executable='listener_diy',
            name='listener_diy'
        )
    ])