import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )

    turtle1_tf_broadcaster = ComposableNodeContainer(
            name='turtle1_tf_broadcaster',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='learning_tf2_cpp',
                    plugin='learning_tf2_cpp::TurtleTfBroadcaster',
                    name='turtle1_tf_bc',
                    parameters=[{'turtle_name': 'turtle1'}])
            ],
            output='screen',
    )

    turtle2_tf_broadcaster = ComposableNodeContainer(
            name='turtle2_tf_broadcaster',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='learning_tf2_cpp',
                    plugin='learning_tf2_cpp::TurtleTfBroadcaster',
                    name='turtle2_tf_bc',
                    parameters=[{'turtle_name': 'turtle2'}])
            ],
            output='screen',
    )

    turtle2_listen_turtle1 = ComposableNodeContainer(
            name='turtle2_listen_turtle1',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='learning_tf2_cpp',
                    plugin='learning_tf2_cpp::TurtleTfListener',
                    name='turtle_tf_ls',
                    parameters=[{'target_frame': 'turtle1'}])
            ],
            output='screen',
    )

    return launch.LaunchDescription([turtlesim_node, turtle1_tf_broadcaster, turtle2_tf_broadcaster, turtle2_listen_turtle1])
