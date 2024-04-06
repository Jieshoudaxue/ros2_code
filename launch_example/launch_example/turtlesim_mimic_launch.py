from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 默认命名空间是 /, 但是这里的命名空间是 turtlesim1
        # 因此所有的计算图源名称都会带上/turtlesim1，例如：
        # 节点名：/turtlesim1/sim
        # topic名: /turtlesim1/turtle1/cmd_vel，/turtlesim1/turtle1/pose
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        # 同上
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        # remap后，mimic 节点会订阅 /turtlesim1/turtle1/pose 话题，
        # 处理后发布 /turtlesim2/turtle1/cmd_vel 话题，从而实现两个乌龟的同步运动
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        ),
        # 延迟5秒，启动一个额外命令，向 /turtlesim1/sim 发送转圈控制命令，让乌龟1转圈，乌龟2会跟随
        TimerAction(
            period=5.0,
            actions=[     
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '-r', '1',
                        '/turtlesim1/turtle1/cmd_vel', 'geometry_msgs/msg/Twist',
                        '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}'
                    ],
                    output='screen'
                )
            ]
        )

    ])