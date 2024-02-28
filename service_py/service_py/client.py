#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import random
import time
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self._client = self.create_client(AddTwoInts, "add_two_ints")

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_req(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    client_node = ServiceClient()

    while True:
        random.seed()
        a = random.randint(0, 2**31-1)
        b = random.randint(0, 2**31-1)
        response = client_node.send_req(a, b)
        client_node.get_logger().info("python client: send {0}(a) + {1}(b), receive {2}(sum)".format(
                                      a, b, response.sum))
        time.sleep(1)
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()