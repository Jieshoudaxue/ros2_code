import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # 两个组件放在一个 container 里，也就是共享一个进程，可以降低负载
    # 通常情况下，开发阶段使用分开进程的方式，生产环境使用这种方式
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            # component_container 是单线程容器，容器内所有组件共享一个线程，没有并发问题，但是效率低
            # component_container_mt 是多线程容器，容器内每个组件都有自己的线程，可以并发处理，效率高，但需要考虑并发风险
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='component_demo',
                    plugin='component_demo::PubComponent',
                    name='pub_component',
                    # 尽管多个组件合并在同一个进程，ros2 也是默认走 DDS 中间件通信
                    # 下面的参数，可以设置组件之间通过 intra-process 通信
                    # 理论上，intra-process 直接传递指针，效率更高
                    # 但是本样例太小了，实际测试无法看出使用 intra-process 的优势
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='component_demo',
                    plugin='component_demo::SubComponent',
                    name='sub_component',
                    # 接收端也要配置 intra-process
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])