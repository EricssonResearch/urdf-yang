from rclpy.node import Node
from std_msgs.msg import String
from xml.dom.minidom import parseString

import re
import json
import rclpy
import xmltodict

from yangson import DataModel
from lxml import etree



class urdf_validator(Node):
    def __init__(self):
        super().__init__('urdf_formatter_node')
        self.subscription = self.create_subscription(String, '/robot_description', self.robot_description_callback, 10)

        self.publisher = self.create_publisher(String, '/yang_instance', 10)
        self.get_logger().info("init done")

    def write_to_file(self, data, filename):
        with open(filename, "w") as f:
            f.write(data)
        self.get_logger().info(f"YANG instance data written to file: {filename}")

    def convert_xml_to_json(self, xml_string):
        # Parse XML string to ordered dictionary
        xml_dict = xmltodict.parse(xml_string)

        # Convert ordered dictionary to JSON string
        json_string = json.dumps(xml_dict, indent=4)

        return json_string


    def validate_yang_data(self, instance_data_path, yang_module_path):
        
        self.get_logger().info("trying to load data model")
        # try:
        # Load YANG data model
        self.get_logger().info("loading data model.......")
        data_model = DataModel.from_file(yang_module_path)

        self.get_logger().info("data model loaded")
        # Load instance data based on file extension
        with open(instance_data_path, 'r') as f:
            if instance_data_path.endswith(".xml"):
                instance_data = etree.parse(f).getroot()
                instance_data_dict = etree.tostring(instance_data).decode()  # Convert to string first
                instance_data_dict = json.loads(instance_data_dict)  # Parse as JSON
            else:
                instance_data = f.read()

            # Validate
        data_model.validate(instance_data_dict if isinstance(instance_data, dict) else instance_data)
        self.get_logger().info("Validation successful!")
        return True

        # except (IOError, etree.XMLSyntaxError, Exception) as e:  # Catch broader exceptions
        #     self.get_logger().info(f"Validation failed: {e}")
        #     return False


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

        # pretty print to file
        parsed_xml = parseString(yang_xml_data)
        yang_xml_data_pretty = parsed_xml.toprettyxml()
        # # write to file
        # self.write_to_file(yang_xml_data_pretty, "yang_content.xacro") # !!!!!!!!!!!!! remove for debug!!!!

        # convert yang xml instance to yang json instance!
        xml_dict = xmltodict.parse(yang_xml_data_pretty)
        # Convert ordered dictionary to JSON string
        json_string = json.dumps(xml_dict, indent=4)

        # debug log & publish
        # yang_xml_msg.data = yang_xml_data
        # self.publisher.publish(yang_xml_msg)
        # self.get_logger().info(yang_xml_data)

        yang_library_data = "system-library-data.json"
        xml_instance_file = "yang_content.xml"

        # Create and use the YangValidator class
        self.validate_yang_data(xml_instance_file, yang_library_data)



def main(args=None):
    rclpy.init(args=args)
    robot_description_subscriber = urdf_validator()
    rclpy.spin(robot_description_subscriber)
    robot_description_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()