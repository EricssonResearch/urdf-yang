#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.declare_parameter('data', 'Default data')
        self.publisher_ = self.create_publisher(String, '/system_description', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Timer callback every 1 second

    def timer_callback(self):
        msg = String()
        msg.data = self.get_parameter('data').get_parameter_value().string_value
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()