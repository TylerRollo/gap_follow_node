#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='reactive_node.py',
            name='reactive_node',
            output='screen',
        )
    ])
