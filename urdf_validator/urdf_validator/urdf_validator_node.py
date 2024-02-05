from rclpy.node import Node
from std_msgs.msg import String
from lxml import etree


import re
import rclpy

class urdf_validator(Node):
    def __init__(self):
        super().__init__('urdf_validator_node')
        self.subscription = self.create_subscription(String, '/robot_description', self.robot_description_callback, 10)
        self.subscription
        self.get_logger().info("init done")

    def robot_description_callback(self, msg):

        robot_description = msg.data
        urdf = str()

        # Use a regular expression to find all content between <yang> and </yang> tags
        # matches = re.findall(r'<yang>(.*?)</yang>', robot_description, re.DOTALL)

        # for match in matches:
        #     # self.get_logger().info(f'Received Robot Description:\n{match}')
        #     urdf = urdf + '' + match


        # self.process_and_publish_content(matches)
        self.process_and_publish_content(robot_description)

    def process_and_publish_content(self, urdf_topic_content):

        # tree = etree.fromstring(urdf_topic_content)
        # etree.strip_tags(tree,etree.Comment)

        # self.get_logger().info(f"Processed Content:\n {urdf_topic_content}")
        self.get_logger().info(urdf_topic_content)

def main(args=None):
    rclpy.init(args=args)
    robot_description_subscriber = urdf_validator()
    rclpy.spin(robot_description_subscriber)
    robot_description_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()