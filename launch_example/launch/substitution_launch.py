from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
# 导入logging模块来获取logger，这是launch原生支持的日志模块
import launch.logging

def generate_launch_description():
    logger = launch.logging.get_logger('main_substitution_launch_debugger')
    # 当由 main_launch.py 启动时，这句会打印两次
    # 当由本文件启动时，这句只打印一次，知道就好
    logger.info('main_substitution_launch.py is started.')

    # 创建命名空间的配置变量
    # 创建是否使用指定红色值的配置变量
    # 创建新的红色背景值的配置变量
    turtlesim_ns = LaunchConfiguration('turtlesim_ns')
    use_provided_red = LaunchConfiguration('use_provided_red')
    new_background_r = LaunchConfiguration('new_background_r')

    # 声明命名空间配置, 是否使用指定红色值, 新的红色背景值的启动参数, 分别对应三个配置变量
    # 使用 DeclareLaunchArgument 申明的启动参数, 允许从外部为这些参数提供值。
    # 在这个例子中, 这三个参数将从 main_launch.py 中传入
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

    # 实例化一个Node, 用于启动 turtlesim 节点
    turtlesim_node = Node(
        package='turtlesim',
        namespace=turtlesim_ns,
        executable='turtlesim_node',
        name='sim'
    )
    # 实例化一个ExecuteProcess, 用于在3秒后生成一个乌龟
    spawn_turtle = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            turtlesim_ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    # 实例化三个ExecuteProcess, 用于启动3秒后改变背景颜色为护眼绿
    change_background_r = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r 199'
        ]], # 这个逗号是必须的，不能删除
        shell=True
    )
    change_background_g = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_g 237'
        ]],
        shell=True
    )
    change_background_b = ExecuteProcess(
        cmd=[[
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_b 204'
        ]],
        shell=True
    )
    # 实例化一个条件ExecuteProcess, 如果外部 main_launch.py 传入的新的红色背景值为240且使用指定红色值，则改变背景颜色为240
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
            'ros2 param set ',
            turtlesim_ns,
            '/sim background_r ',
            new_background_r
        ]],
        shell=True
    )

    # 前面都是定义各种变量和实例化各种对象，这里是返回最终的LaunchDescription对象
    # 如果launch文件比较复杂，就推荐这种方式，便于修改和抽象，而不是直接把内容在LaunchDescription中展开
    return LaunchDescription([
        # 先申明三个启动参数
        turtlesim_ns_launch_arg,
        use_provided_red_launch_arg,
        new_background_r_launch_arg,
        # 启动 turtlesim 节点
        turtlesim_node,
        # 3秒后生成一个乌龟，并把背景色改为护眼绿
        TimerAction(
            period=3.0,
            actions=[spawn_turtle,
                    change_background_r,
                    change_background_g,
                    change_background_b],
        ),
        # 5秒后，如果main_launch.py传入的参数符合条件，则改变背景颜色的red值
        TimerAction(
            period=5.0,
            actions=[change_background_r_conditioned],
        )        
    ])