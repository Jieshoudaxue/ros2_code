#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self._service = self.create_service(AddTwoInts, "add_two_ints", self.server_callback)

    def server_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("python server: receive {0}(a) + {1}(b), return {2}(sum)".format(
                                request.a, request.b, response.sum))
        return response

def main(args=None):
    rclpy.init(args=args)

    server_node = ServiceServer()
    rclpy.spin(server_node)
    
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()