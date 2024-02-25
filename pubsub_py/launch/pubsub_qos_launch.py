from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_py',
            namespace='qos',
            executable='talker_qos',
            name='talker_qos'
        ),
        TimerAction(
            period=3.0,   # test qos durability:TRANSIENT_LOCAL
            # period=0.0,
            actions=[
                Node(
                    package='pubsub_py',
                    namespace='qos',
                    executable='listener_qos',
                    name='listener_qos'
                )
            ]
        )
    ])