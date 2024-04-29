from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='turtle1_tf_bc',
            parameters=[
                {'turtle_name': 'turtle1'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='turtle2_tf_bc',
            parameters=[
                {'turtle_name': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='turtle_tf_ls',
            parameters=[
                {'target_frame': 'turtle1'}
            ]
        ),
    ])