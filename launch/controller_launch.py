from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    params_file = PathJoinSubstitution(
        [FindPackageShare('scroll_approach_project'), 'config', 'params.yaml']
    )

    
    bridge_params = {
        'port': '/dev/ttyACM0',
        'baudrate': 115200
    }

    return LaunchDescription([
        
        Node(
            package='scroll_approach_project',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[bridge_params]
        ),

        Node(
            package='scroll_approach_project',
            executable='scroll_approach_controller',  
            name='scroll_approach_controller',
            output='screen',
            parameters=[params_file]
        ),
    ])