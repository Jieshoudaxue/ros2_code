#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time

class HelloRos2(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("start hello_ros2_py node !!")
        
    def run(self):
        try:
            while rclpy.ok():
                self.get_logger().info("hello ros2 in python !!")
                time.sleep(1)
        except KeyboardInterrupt:
            self.get_logger().info("Node was interrupted, shutting down...")
            
            
def main(args=None):
    rclpy.init(args=args)
    node = HelloRos2("hello_ros2_py")
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()