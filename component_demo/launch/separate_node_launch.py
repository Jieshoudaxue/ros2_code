import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    pub_container = ComposableNodeContainer(
            name='pub_container',
            namespace='',
            package='rclcpp_components',
            # component_container_mt
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='component_demo',
                    plugin='component_demo::PubComponent',
                    name='pub_component')
            ],
            output='screen',
    )

    sub_container = ComposableNodeContainer(
            name='sub_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='component_demo',
                    plugin='component_demo::SubComponent',
                    name='sub_component')
            ],
            output='screen',
    )

    return launch.LaunchDescription([pub_container, sub_container])