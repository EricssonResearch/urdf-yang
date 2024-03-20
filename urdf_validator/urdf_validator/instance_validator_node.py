from rclpy.node import Node
from std_msgs.msg import String

import re
import json
import rclpy
import xmltodict
from lxml import etree
from yangson import DataModel
from xml.dom.minidom import parseString


class instance_validator(Node):
    def __init__(self):
        super().__init__('urdf_validator_node')
        self.subscription = self.create_subscription(String, '/robot_description', self.robot_description_callback, 10)
        self.subscription
        self.get_logger().info("init done")


    def yang_content_extract(self, robot_description):
        
        yang_xml_data = str()

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

        # pretty print to file
        parsed_xml = parseString(yang_xml_data)
        yang_xml_data_pretty = parsed_xml.toprettyxml()

        return yang_xml_data_pretty

    
    def write_to_file(self, data, filename):
        with open(filename, "w") as f:
            f.write(data)
        self.get_logger().info(f"{data} is written to file: {filename}")

    
    def json_validation_against_yang(self, json_instance, yang_library_data, yang_modules_folder):

        self.get_logger().info("loading data model...")
        data_model = DataModel.from_file(yang_library_data, [yang_modules_folder])
        self.get_logger().info("data model loaded")

        # self.get_logger().info("printing data model...")
        # self.get_logger().info(data_model.ascii_tree(), end='') ## end nem jó valamiért


        with open(json_instance, 'r') as j:
            json_instance_content = json.loads(j.read())

        self.get_logger().info("loading instance...")
        instance_data = data_model.from_raw(json_instance_content)

        self.get_logger().info("instance loaded")

        instance_data.validate()  # No output means that the validation was successful.

        if instance_data.validate() == None:
            self.get_logger().info("\n data instance is VALID!")
        else:
            self.get_logger().info("\n data instance is INVALID!")

        return

    
    def robot_description_callback(self, msg):

        robot_description = msg.data
        yang_xml_data_pretty = str()

        yang_xml_data_pretty = self.yang_content_extract(robot_description)

        #### TMP file saving: yang data in xml format
        self.write_to_file(yang_xml_data_pretty, '/home/martzi/component_descr_ws/src/yang_tools/yang_content.xml')

        xml_dict = xmltodict.parse(yang_xml_data_pretty)

        # Convert ordered dictionary to JSON string
        json_string = json.dumps(xml_dict, indent=4)
        
        #### TMP file saving: yang data in xml format
        self.write_to_file(json_string, '/home/martzi/component_descr_ws/src/yang_tools/yang_instance_data.json')

        # self.process_and_publish_content(matches)
        # self.process_and_publish_content(robot_description)
        self.get_logger().info("Done!")

        yang_library_data = "/home/martzi/component_descr_ws/src/yang_tools/system-library-data.json"
        yang_modules_folder = "/home/martzi/component_descr_ws/src/yang_tools/yang_modules"
        xml_instance_file = "/home/martzi/component_descr_ws/yang_content.xml"
        # json_instance_file = "/home/martzi/component_descr_ws/src/yang_tools/yang_instance_data.json"
        json_instance_file = "/home/martzi/component_descr_ws/src/yang_tools/instance_data.json"

        # Create and use the YangValidator class
        self.json_validation_against_yang(json_instance_file, yang_library_data, yang_modules_folder)

def main(args=None):
    rclpy.init(args=args)
    robot_description_subscriber = instance_validator()
    rclpy.spin(robot_description_subscriber)
    robot_description_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()