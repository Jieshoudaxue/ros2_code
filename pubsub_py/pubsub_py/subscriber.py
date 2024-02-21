#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self._subscriber = self.create_subscription(String, "hello_topic", self.topic_callback, 10)

    def topic_callback(self, msg):
        self.get_logger().info("subscribe: %s" % msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    sub_node = Subscriber()
    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()