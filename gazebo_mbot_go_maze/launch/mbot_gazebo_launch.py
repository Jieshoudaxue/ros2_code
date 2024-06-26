import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name='gazebo_mbot_go_maze' #<--- CHANGE ME
    world_file_path = 'worlds/maze_room.world'    
    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)  
    rviz_path = os.path.join(pkg_path, 'rviz/mbot_gazebo.rviz')
    print(rviz_path)

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'urdf','mbot_base.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    mbot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
                )]), launch_arguments={'world':world_path}.items()
             )

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mbot',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')


    rviz_node = Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         arguments=['-d', rviz_path])

    # Launch them all!
    return LaunchDescription([
        mbot,
        gazebo,
        spawn_entity,
        TimerAction(
            period=3.0,
            actions=[rviz_node],
        )
    ])
