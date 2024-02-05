import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    params = {'use_nominal_extrinsics': 'true', 'add_plug': 'true'}

    xacro_path = '/home/martzi/componentdescr_ws/install/realsense2_description/share/realsense2_description/urdf/test_d455_camera.urdf.xacro'
    # kell egy xacro fuggveny ami tmp fajl nelkul atalakitja a xacro-t urdf-re
    urdf_path = '/home/martzi/componentdescr_ws/install/realsense2_description/share/realsense2_description/urdf/tmp.urdf.xacro'



    realsense_model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments=[urdf_path]
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}]
        )

    ur_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_description'), 'launch'),
            '/view_ur.launch.py']),
        launch_arguments={'ur_type': 'ur3e', 'add_controlbox': 'true'}.items(),
    )

    return LaunchDescription([
        ur_robot_node,
        realsense_model_node,

    ])