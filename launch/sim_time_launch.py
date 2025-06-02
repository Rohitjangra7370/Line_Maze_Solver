#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Sync with Gazebo time'
        )
    ])