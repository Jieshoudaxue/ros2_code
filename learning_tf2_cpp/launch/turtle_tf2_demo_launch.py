import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    # 启动turtlesim节点，这个是基础环境
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )
    # 启动turtle1的tf广播节点，广播turtle1的tf
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
    # 启动turtle2的tf广播节点，广播turtle2的tf
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
    # 启动rviz2，加载turtlesim的 world 和两个小乌龟坐标系的可视化配置
    turtlesim_world_rviz_config = os.path.join(
        get_package_share_directory('learning_tf2_cpp'),
        'config',
        'turtle_tf.rviz'
    )
    turtlesim_world_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', turtlesim_world_rviz_config]
    )
    # 启动tf监听节点，监听在 turtle2 坐标系下的 turtle1 的 tf，并转换为 turtle2 的控制指令
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

    return launch.LaunchDescription([turtlesim_node, turtle1_tf_broadcaster, turtle2_tf_broadcaster, turtlesim_world_rviz_node, turtle2_listen_turtle1])
