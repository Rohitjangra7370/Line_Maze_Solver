#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('line_maze_solver')
    launch_dir = os.path.join(pkg_share, 'launch')
    
    return LaunchDescription([
        # Launch Arguments

        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'sim_time_launch.py']),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'joint_state_publisher_launch.py']),
        ),
        # IncludeLaunchDescription(
        #     PathJoinSubstitution([launch_dir, 'joint_state_publisher_gui_launch.py']),
        # ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'line_follower_launch.py']),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'image_processor_launch.py']),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir,'robot_state_publisher_launch.py']),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'gazebo_launch.py']),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'rviz_launch.py'])
        )

    ])