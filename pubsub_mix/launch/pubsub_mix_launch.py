from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_mix',
            namespace='mix',
            executable='publish_address_book',
            name='publish_address_book'
        ),
        Node(
            package='pubsub_mix',
            namespace='mix',
            executable='subscribe_address_book.py',
            name='subscribe_address_book'
        )
    ])