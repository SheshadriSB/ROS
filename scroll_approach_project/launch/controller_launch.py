#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Package name
    pkg_name = 'scroll_approach_project'

    # Correct way to reference params.yaml in package share
    params_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'params.yaml'
    ])

    return LaunchDescription([
        # Deadwheel bridge (odometry simulation)
        Node(
            package=pkg_name,
            executable='deadwheel_bridge_node',
            name='deadwheel_bridge',
            output='screen',
            parameters=[params_file],
        ),

        # Motor bridge (cmd_vel simulation)
        Node(
            package=pkg_name,
            executable='motor_bridge_node',
            name='motor_bridge',
            output='screen',
            parameters=[params_file],
        ),

        # Main controller
        Node(
            package=pkg_name,
            executable='scroll_approach_controller',
            name='scroll_approach_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])