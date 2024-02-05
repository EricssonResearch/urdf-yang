from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urdf_validator',
            executable='urdf_validator_node',
            name='urdf_validator_node',
            output='screen',
        ),
    ])