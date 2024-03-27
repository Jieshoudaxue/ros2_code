import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',   # component_container_mt
            composable_node_descriptions=[
                ComposableNode(
                    package='component_demo',
                    plugin='component_demo::PubComponent',
                    name='pub_component',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='component_demo',
                    plugin='component_demo::SubComponent',
                    name='sub_component',
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])