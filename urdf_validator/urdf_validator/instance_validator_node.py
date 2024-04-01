from rclpy.node import Node
from std_msgs.msg import String

import re
import json
import rclpy
import xmltodict
from lxml import etree
from yangson import DataModel
from xml.dom.minidom import parseString
import xml.etree.ElementTree as ET


class instance_validator(Node):
    def __init__(self):
        super().__init__('urdf_validator_node')
        self.subscription = self.create_subscription(String, '/robot_description', self.robot_description_callback, 10)
        self.get_logger().info("init done")

    def xml_to_json(self, element):
        result = {}
        for child in element:
            tag = child.tag
            if '}' in tag:
                prefix, tag = tag.split('}')
                tag = f"{prefix[1:]}:{tag}"
            if child:
                if len(child) > 1 or child[0].tag != child[-1].tag:
                    if tag not in result:
                        result[tag] = []
                    result[tag].append(self.xml_to_json(child))
                else:
                    if tag not in result:
                        result[tag] = {}
                    for subchild in child:
                        subtag = subchild.tag
                        if '}' in subtag:
                            subprefix, subtag = subtag.split('}')
                            subtag = f"{subprefix[1:]}:{subtag}"
                        # print("tag: ", tag, "subtag: ", subtag, "result: ", subchild.text)
                        result[tag][subtag] = subchild.text
            else:
                result[tag] = child.text
        return result


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
        self.get_logger().info(f"is written to file: {filename}")

    
    def json_validation_against_yang(self, json_instance, yang_library_data, yang_modules_folder):

        self.get_logger().info("loading data model...")
        data_model = DataModel.from_file(yang_library_data, [yang_modules_folder])
        self.get_logger().info("data model loaded")

        self.get_logger().info("printing data model...\n")
        self.get_logger().info( data_model.ascii_tree()) 


        with open(json_instance, 'r') as j:
            json_instance_content = json.loads(j.read())

        self.get_logger().info("loading instance...")
        instance_data = data_model.from_raw(json_instance_content)

        instance_data.validate()  # No output means that the validation was successful.

        if instance_data.validate() == None:
            self.get_logger().info("YANG data instance is VALID!")
        else:
            self.get_logger().info("YANG data instance is INVALID!")

        return

    def replace_namespace_prefixes(self, json_data, namespace_mapping):
        new_json_data = {}
        for key, value in json_data.items():
            new_key = self.replace_namespace_prefix(key, namespace_mapping)
            if isinstance(value, dict):
                new_value = self.replace_namespace_prefixes(value, namespace_mapping)
            elif isinstance(value, list):
                new_value = [self.replace_namespace_prefixes(item, namespace_mapping) for item in value]
            else:
                new_value = value
            new_json_data[new_key] = new_value
        return new_json_data

    def replace_namespace_prefix(self, key, namespace_mapping):
        for uri, prefix in namespace_mapping.items():
            if key.startswith(uri):
                return key.replace(uri, prefix + ":")
        return key
    
    def robot_description_callback(self, msg):
        
        ### TMP solution - needs to be parsed from XML directly
        namespace_mapping = {
            "urn:ietf:params:xml:ns:yang:robot-network-topology:": "robot-network-topology",
            "urn:ietf:params:xml:ns:yang:robot-network:": "robot-network",
            "urn:ietf:params:xml:ns:yang:layer1:": "layer1",
            "urn:ietf:params:xml:ns:yang:layer2:": "layer2",
            "urn:ietf:params:xml:ns:yang:robot-control-loop:": "robot-control-loop"
        }

        yang_library_data = "src/yang_tools/system-library-data.json"
        yang_modules_folder = "src/yang_tools/yang_modules"
        yang_xml_instance = "src/yang_tools/yang_content.xml"
        json_instance_file = "src/yang_tools/instance_data.json"

        robot_description = msg.data
        yang_xml_data_pretty = str()

        yang_xml_data_pretty = self.yang_content_extract(robot_description)

        # robot description model urdf saved to file
        self.write_to_file(yang_xml_data_pretty, yang_xml_instance)
        self.get_logger().info("robot_description topic is saved!")

        tree = ET.parse(yang_xml_instance)
        root = tree.getroot()

        root.tag="robot-network:networks"

        self.get_logger().info("xml instance topic is loaded!")
        
        json_result = json.dumps({root.tag: [self.xml_to_json(root)]}, indent=4)

        # Parse JSON data
        json_data = json.loads(json_result)

        # Replace namespace prefixes
        new_json_data = self.replace_namespace_prefixes(json_data, namespace_mapping)

        # Convert back to JSON string
        new_json_string = json.dumps(new_json_data, indent=4)

    # TMP FOR DEBUG!
        self.write_to_file(new_json_string, json_instance_file)
        self.get_logger().info("json instance data is saved!")

        # self.process_and_publish_content(matches)
        # self.process_and_publish_content(robot_description)

        # Create and use the YangValidator class
        self.json_validation_against_yang(json_instance_file, yang_library_data, yang_modules_folder)

        self.get_logger().info("Validation is Done!")


def main(args=None):
    rclpy.init(args=args)
    robot_description_subscriber = instance_validator()
    rclpy.spin(robot_description_subscriber)
    robot_description_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()