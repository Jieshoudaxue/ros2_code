from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_py',
            namespace='qos',
            executable='talker_qos',
            name='talker_qos'
        ),
        Node(
            package='pubsub_py',
            namespace='qos',
            executable='listener_qos',
            name='listener_qos'
        )
    ])