#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    pkg_path = get_package_share_directory('line_maze_solver')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(pkg_path,'worlds','line_follower.world')

    return LaunchDescription([



        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'maze_bot',
                '-x', '9.464182',
                '-y', '13.545395',
                '-z', '0.034947',
                '-robot_namespace', '/'
            ],
            output='screen',
            condition=IfCondition(use_gazebo)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': PathJoinSubstitution([pkg_path, 'worlds', 'line_follower.world'])}.items()   # If you take 'world_path' as a argument, it will not work.
        ),
    ])