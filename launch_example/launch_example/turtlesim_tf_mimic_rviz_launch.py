import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)
from launch.events import Shutdown
from launch.actions import (ExecuteProcess, LogInfo, RegisterEventHandler)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():
   # 使用 IncludeLaunchDescription 来包含 turtle_tf2_py 包中的 turtle_tf2_demo.launch.py 文件
   # 该文件会启动一个窗口，里面有两个乌龟，turtle2始终追着turtle1运动，实现这个效果需要 TF 的知识，这里并不深究
   turtlesim_world_1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('turtle_tf2_py'), 'launch'),
         '/turtle_tf2_demo.launch.py'])
      )
   
   # 启动第二个窗口，其背景色通过加载全局参数文件配置成护眼绿
   # 在下面的 mimic_node 的作用下 ，这个窗口的乌龟将与第一个窗口的turtle2同步运动
   turtlesim_world_2_param = os.path.join(
      get_package_share_directory('launch_example'),
      'config',
      'turtlesim_param.yaml'
      )
   turtlesim_world_2 = Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         parameters=[turtlesim_world_2_param]
      )
   # 为了防止两个窗口的节点名字冲突，这里使用 PushRosNamespace 来给第二个窗口的节点加上一个命名空间名turtlesim2
   # PushRosNamespace可以在不修改launch文件的情况下，给节点加上命名空间，避免重名，非常高效
   turtlesim_world_2_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('turtlesim2'),
         turtlesim_world_2,
      ]
   )
   # mimic_node 会订阅第一个窗口的turtle2乌龟的位置信息，然后控制第二个窗口的乌龟做同步运动
   mimic_node = Node(
         package='turtlesim',
         executable='mimic',
         name='mimic',
         remappings=[
            ('/input/pose', '/turtle2/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
         ]
      )
   # 针对第一个窗口，启动rviz，监听两个turtle的TF信息，并可视化显示
   turtlesim_world_1_rviz_config = os.path.join(
      get_package_share_directory('turtle_tf2_py'),
      'rviz',
      'turtle_rviz.rviz'
      )
   turtlesim_world_1_rviz_node = Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', turtlesim_world_1_rviz_config]
      )
   # 由于第一个窗口的turtle2乌龟会追着turtle1乌龟运动，而第二个窗口的turtle1乌龟会与第一个窗口的turtle2同步运动
   # 因此需要给第一个窗口的turtle1乌龟添加额外的运动控制命令，不然都不动了
   # 这里是让第一个窗口的turtle1乌龟做圆周运动
   draw_cycle = ExecuteProcess(
      cmd=[
         'ros2', 'topic', 'pub', '-r', '1',
         '/turtle1/cmd_vel', 'geometry_msgs/msg/Twist',
         '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}'
      ],
      output='screen'
   )

   return LaunchDescription([
      turtlesim_world_1,
      turtlesim_world_2_with_namespace,
      mimic_node,
      turtlesim_world_1_rviz_node,
      # 依次启动这些节点，最后启动draw_cycle
      RegisterEventHandler(
         OnProcessStart(
               target_action=turtlesim_world_1_rviz_node,
               on_start=[
                  LogInfo(msg='Turtlesim started, spawning turtle'),
                  draw_cycle
               ]
         )
      ),
      RegisterEventHandler(
         OnProcessExit(
               target_action=turtlesim_world_1_rviz_node,
               on_exit=[
                  LogInfo(msg=(EnvironmentVariable(name='USER'),
                           ' closed the turtlesim window')),
                  EmitEvent(event=Shutdown(
                     reason='Window closed'))
               ]
         )
      ),
      RegisterEventHandler(
         OnShutdown(
               on_shutdown=[LogInfo(
                  msg=['Launch was asked to shutdown: ',
                     LocalSubstitution('event.reason')]
               )]
         )
      ),      
   ])