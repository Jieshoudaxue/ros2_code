from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    # 定义一个字典，包含背景颜色的红色值
    colors = {
        'background_r': '240'
    }
    # 通过引用launch_example 包内的 launch 文件夹下的 substitution_launch.py 来启动
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_example'),
                    'launch',
                    'substitution_launch.py'
                ])
            ]),
            # 创建一个字典，使用.items()获取字典的键值对组成的元组列表，向 substitution_launch.py 传递三个参数
            # TextSubstitution 是ROS 2 launch文件中的一个功能，它允许在launch过程中动态生成字符串
            # TextSubstitution 支持从环境变量中读取值，也支持将多个字符串拼接起来
            launch_arguments={
                'turtlesim_ns': 'turtlesim',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])