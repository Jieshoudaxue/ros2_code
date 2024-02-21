#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self._publisher = self.create_publisher(String, "hello_topic", 10)
        self._timer = self.create_timer(0.5, self.timer_callback)
        self._i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "hello, i am fine in python! %d" % self._i
        self._publisher.publish(msg)
        self.get_logger().info("publish: %s" % msg.data)
        self._i += 1


def main(args=None):
    rclpy.init(args=args)

    pub_node = Publisher()
    rclpy.spin(pub_node)
    
    pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()