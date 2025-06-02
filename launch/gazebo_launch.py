#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    pkg_path = get_package_share_directory('line_maze_solver')
    world_path = os.path.join(pkg_path,'worlds','line_world.world')

    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': 'world_path'}.items()   # If you take 'world_path' as a argument, it will not work.
        )
    return LaunchDescription([
        
        gazebo,


        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'maze_bot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.5',
                '-robot_namespace', '/'
            ],
            output='screen',
            condition=IfCondition(use_gazebo)
        ),
    ])