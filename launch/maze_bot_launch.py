#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('line_maze_solver')
    
    # Paths to your files
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Process the xacro file to generate robot description
    robot_description_config = Command(['xacro ', xacro_file,])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true' if LaunchConfiguration('use_gazebo') else 'false',
            description='Sync with Gazebo time'
        ),
        
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='Start Gazebo simulation'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 for visualization'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time
            }]
        ),

        # Joint State Publisher GUI (for manual joint control)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Include Gazebo (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            condition=IfCondition(use_gazebo)
        ),

        # Spawn Robot in Gazebo (conditional)
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

        # RViz2 (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz)
        ),
    ])