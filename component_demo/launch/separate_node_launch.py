import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # 两个组件放在两个 container 里，也就是分别在两个进程，这种状况比较容易排查问题，当然负载也要高一些
    # 通常情况下，开发阶段使用这种方式，生产环境使用合并进程的方式
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