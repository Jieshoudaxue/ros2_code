import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
   turtlesim_world_1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('turtle_tf2_py'), 'launch'),
         '/turtle_tf2_demo.launch.py'])
      )
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_example'), 'launch_senior'),
         '/turtlesim_world_2_launch.py'])
      )
   turtlesim_world_2_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('turtlesim2'),
         turtlesim_world_2,
      ]
   )
   mimic_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_example'), 'launch_senior'),
         '/mimic_launch.py'])
      )
   rviz_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_example'), 'launch_senior'),
         '/turtlesim_rviz_launch.py'])
      )

   return LaunchDescription([
      turtlesim_world_1,
      turtlesim_world_2_with_namespace,
      mimic_node,
      rviz_node
   ])