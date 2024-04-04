from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    turtlesim_ns_launch_arg = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value=''
    )
    use_provided_red_launch_arg = DeclareLaunchArgument(
        'use_provided_red',
        default_value='False'
    )
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='200'
    )

    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    change_background_r = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            '120'
        ]],
        shell=True
    )
    change_background_g = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_g ',
            '237'
        ]],
        shell=True
    )
    change_background_b = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_b ',
            '204'
        ]],
        shell=True
    )    
    change_background_r_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                new_background_r,
                ' == 240',
                ' and ',
                use_provided_red
            ])
        ),
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )
    # 该launch文件的功能同 main_launch.py + substitution_launch.py
    # 上面的代码注释同 substitution_launch.py，我们重点解释下面的 event handler
    ########################################################################

    # ROS2 launch 的事件处理器(event handler)是一种机制，它允许用户自定义对特定事件的响应行为。
    # 当进程状态发生变化时（例如，节点启动、节点关闭、进程死亡等），这些变化会触发自定义的响应行为。
    # 使用事件处理器，launch 系统具备了高度的灵活性和可扩展性，使得开发者可以根据不同的运行时情况和需求，定制处理流程和逻辑。
    # 这对于构建稳健、可靠的 ROS 2 应用程序至关重要。
    return LaunchDescription([
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        turtlesim_node,
        # RegisterEventHandler 用于注册一个事件处理器，这里注册了五个事件处理器
        RegisterEventHandler(
            # event_handlers 的 OnProcessStart 用于处理进程启动事件，当进程启动时触发响应行为
            OnProcessStart(
                # target_action 指定了事件处理器要监听的目标动作，这里是拉起 turtlesim_node
                target_action=turtlesim_node,
                # on_start 是一个列表，用来定义进程启动后的响应行为
                # 这里的行为是打印一条日志，然后执行 spawn_turtle
                on_start=[
                    LogInfo(msg='Turtlesim started, spawning turtle'),
                    spawn_turtle
                ]
            )
        ),
        RegisterEventHandler(
            # event_handlers 的 OnProcessIO 用于处理进程输入输出事件，当进程有输入/输出（如标准输出）时触发响应行为
            OnProcessIO(
                # 这里要监听的事件是 spawn_turtle 的标准输出
                target_action=spawn_turtle,
                # on_stdout 是一个函数，这里用 lambda表达式打印 spawn_turtle 的标准输出
                # lambda表达式是一个匿名函数，用于简单的函数定义，其中 event 是函数参数，返回值是LogInfo对象
                # event.text是spawn_turtle的标准输出，event.text.decode()将字节流解码为字符串
                on_stdout=lambda event: LogInfo(
                    msg='Spawn request says "{}"'.format(
                        event.text.decode().strip())
                )
            )
        ),
        RegisterEventHandler(
            # event_handlers 的 OnExecutionComplete 用于处理动作执行完成事件，当动作执行完成时触发响应行为
            OnExecutionComplete(
                # 这里的目标动作仍是 spawn_turtle
                target_action=spawn_turtle,
                # on_completion 是一个列表，用来定义动作执行完成后的响应行为
                # 这里的行为是打印一条日志，然后执行将背景色改为护眼绿，然后根据参数有条件的将背景色的red值改为240
                on_completion=[
                    LogInfo(msg='Spawn finished'),
                    change_background_r,
                    change_background_g,
                    change_background_b,
                    TimerAction(
                        period=2.0,
                        actions=[change_background_r_conditioned],
                    )                    
                ]
            )
        ),
        RegisterEventHandler(
            # event_handlers 的 OnProcessExit 用于处理进程退出事件，当进程退出时触发响应行为
            OnProcessExit(
                # 这里要监听的事件是 turtlesim_node 的退出
                target_action=turtlesim_node,
                # on_exit 是一个列表，用来定义进程退出后的响应行为
                # 这里的行为是打印一条日志，然后发出 Shutdown 系统关闭事件
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
        RegisterEventHandler(
            # event_handlers 的 OnShutdown 用于处理系统关闭事件，当系统收到关闭请求时触发响应行为
            OnShutdown(
                # on_shutdown 是一个列表，用来定义系统关闭后的响应行为
                # 这里的行为是打印一条日志，日志中包含了系统关闭的原因
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])