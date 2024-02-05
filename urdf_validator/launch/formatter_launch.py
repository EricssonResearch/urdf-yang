from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urdf_validator',
            executable='urdf_formatter_node',
            name='urdf_formatter_node',
            output='screen',
        ),
    ])