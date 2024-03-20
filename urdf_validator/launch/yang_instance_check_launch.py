from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urdf_validator',
            executable='instance_validator',
            name='instance_validator_node',
            output='screen',
        ),
    ])