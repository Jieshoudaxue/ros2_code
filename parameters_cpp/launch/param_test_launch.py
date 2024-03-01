from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parameters_cpp',
            namespace='cpp',
            executable='param_test',
            name='param_test',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"demo_param": "set in launch !!"}
            ]
        )
    ])