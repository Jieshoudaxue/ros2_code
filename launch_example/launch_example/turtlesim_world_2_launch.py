import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   # Tips: 使用 yaml 文件配置节点参数的方法
   # 加载全局参数文件，设置窗口背景色为护眼绿
   turtlesim_world_2_param = os.path.join(
      get_package_share_directory('launch_example'),
      'config',
      'turtlesim_param.yaml'
      )

   return LaunchDescription([
    Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         parameters=[turtlesim_world_2_param]
      )
   ])
