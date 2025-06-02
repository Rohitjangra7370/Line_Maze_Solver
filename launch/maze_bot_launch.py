#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('line_maze_solver')
    
    # Paths to your files
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_dir = os.path.join(pkg_share, 'launch')
    
    # Process the xacro file to generate robot description
    robot_description_config = Command(['xacro ', xacro_file,])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true' if LaunchConfiguration('use_gazebo') else 'false',
            description='Sync with Gazebo time'
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

        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'gazebo_launch.py']),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'rviz_launch.py'])
        )

    ])