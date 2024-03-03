from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output="screen",
            emulate_tty=True
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='parameters_cpp',
                    namespace='cpp',
                    executable='param_monitor',
                    name='param_monitor',
                    output="screen",
                    emulate_tty=True
                )
            ]
        )

    ])