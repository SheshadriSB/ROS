from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('scroll_approach_project'), 'config', 'params.yaml']
    )

    return LaunchDescription([
        Node(
            package='scroll_approach_project',
            executable='fake_odom_node',
            name='fake_odom_node',
            output='screen'
        ),
        Node(
            package='scroll_approach_project',
            executable='controller_node',
            name='scroll_approach_controller',
            output='screen',
            parameters=[params_file]
        )
    ])