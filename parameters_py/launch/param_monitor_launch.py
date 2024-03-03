from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parameters_py',
            namespace='python',
            executable='param_monitor',
            name='param_monitor',
            output="screen",
            emulate_tty=True
        )
    ])