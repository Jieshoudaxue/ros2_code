from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='polygon_base',
            executable='area_node',
            name='area_node',
            output="screen",
            emulate_tty=True
        )
    ])