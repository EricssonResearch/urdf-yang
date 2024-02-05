from rclpy.node import Node
from std_msgs.msg import String
from lxml import etree
from xml.dom.minidom import parseString

import re
import rclpy

class urdf_validator(Node):
    def __init__(self):
        super().__init__('urdf_formatter_node')
        self.subscription = self.create_subscription(String, '/robot_description', self.robot_description_callback, 10)
        self.subscription

        self.publisher = self.create_publisher(String, '/yang_instance', 10)
        self.get_logger().info("init done")

    def robot_description_callback(self, msg):

        robot_description = msg.data
        yang_xml_data = str()
        yang_xml_msg = String()

        tree = etree.fromstring(robot_description)    
        etree.strip_tags(tree, etree.Comment)
        urdf_nocomment = etree.tostring(tree)

        parsed_xml = parseString(urdf_nocomment)
        xml_pretty_str = parsed_xml.toprettyxml()

        # use first 2 lines (xml tag + namespaces) to start with
        lines = xml_pretty_str.splitlines()
        urdf_first_two_lines = ' '.join(lines[:2])

        # load namespace into yang content to start with
        yang_xml_data = urdf_first_two_lines

        # Use a regular expression to find all content between <yang> and </yang> tags
        matches = re.findall(r'<yang>(.*?)</yang>', robot_description, re.DOTALL)

        for match in matches:
            # self.get_logger().info(f'Received Robot Description:\n{match}')
            yang_xml_data = yang_xml_data + '' + match

        # close the opening robot tag manually
        yang_xml_data = yang_xml_data + '</robot>'

        # parsed_xml = parseString(yang_xml_data)
        # yang_xml_data_pretty = parsed_xml.toprettyxml()

        yang_xml_msg.data = yang_xml_data
        self.publisher.publish(yang_xml_msg)

        self.get_logger().info(yang_xml_data)

def main(args=None):
    rclpy.init(args=args)
    robot_description_subscriber = urdf_validator()
    rclpy.spin(robot_description_subscriber)
    robot_description_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()