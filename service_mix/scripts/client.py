#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import random
import time
from rclpy.node import Node

from service_mix.srv import Circle

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self._client = self.create_client(Circle, "circle")

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_req(self, pr):
        req = Circle.Request()
        req.radius = pr
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    client_node = ServiceClient()

    while True:
        random.seed()
        radius = random.randint(0, 100)
        response = client_node.send_req(radius)
        client_node.get_logger().info("python client: send radius: {0}, receive area: {1}".format(
                                        radius, response.area))
        time.sleep(1)
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()