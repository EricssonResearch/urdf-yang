import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument



import xml.etree.ElementTree as ET


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available
        return None

# Function to find and remove an element by tag
def remove_elements_by_tag(root, tag):
    # Find all parent elements
    for parent in root.findall(".//*"):
        # Remove child elements with the matching tag
        for elem in list(parent):
            if elem.tag == tag or tag in elem.tag:
                parent.remove(elem)

# Function to remove network-related elements
def clean_xml(root):
    tags_to_remove = ['{urn:ietf:params:xml:ns:yang:ietf-network-topology}termination-point', 
                      '{urn:ietf:params:xml:ns:yang:ietf-network-topology}link',
                      '{urn:ietf:params:xml:ns:yang:ietf-network}node',
                      '{urn:ietf:params:xml:ns:yang:ietf-network}supporting-network',
                      '{urn:ietf:params:xml:ns:yang:device-layer}device-layer-node-attributes',
                      '{urn:ietf:params:xml:ns:yang:network-layer}network-layer-node-attributes',
                      'node-id',
                      'network-id',
                      'network-types'
                      ]
    
    # Remove elements based on the tags list
    for tag in tags_to_remove:
        remove_elements_by_tag(root, tag)

def generate_launch_description():

    config_description_path = os.path.join(
        get_package_share_directory('example_config'))

    xacro_file = os.path.join(config_description_path,
                              'urdf',
                              'system_config.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    system_description_config = doc.toxml()

    # print("original system_description: ", system_description_config)
    ############## extracting pure urdf from /robot_description topic

    # Parse the system_description_config XML
    ET.register_namespace('al', 'urn:ietf:params:xml:ns:yang:application-layer')
    ET.register_namespace('nl', 'urn:ietf:params:xml:ns:yang:network-layer')
    ET.register_namespace('dl', 'urn:ietf:params:xml:ns:yang:device-layer')
    ET.register_namespace('nw', 'urn:ietf:params:xml:ns:yang:ietf-network')
    ET.register_namespace('nt', 'urn:ietf:params:xml:ns:yang:ietf-network-topology')
    ET.register_namespace('eth', 'urn:ietf:params:xml:ns:yang:ethernet-port')
    ET.register_namespace('usb', 'urn:ietf:params:xml:ns:yang:usb-port')
    root = ET.fromstring(system_description_config)

    clean_xml(root)
    # Output the cleaned XML with proper formatting
    cleaned_xml = ET.tostring(root, encoding='unicode', method='xml')

    pattern = r"</?(node|network)>"
    result = re.sub(pattern, '', cleaned_xml)

    # print("robot_description: ", result)

    robot_description = {'robot_description': result}

    robot_description_semantic_config = load_file('example_config',
                                                  os.path.join('config', 'example_config_connections.sdf'))
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}


    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('example_config'), 'config', 'rviz_config.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    node_system_publisher = Node(
        package='example_config',
        executable='publisher_node',
        name='publisher_node',
        output='screen',
        parameters=[{'data': LaunchConfiguration('data')}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return LaunchDescription([
        DeclareLaunchArgument('data', default_value=system_description_config, description='Data to be published'),
        joint_state_publisher_node,
        node_system_publisher,
        node_robot_state_publisher,
        rviz_node
    ])
