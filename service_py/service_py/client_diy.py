#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import time
from rclpy.node import Node

from diy_interface.srv import QuestionAndAnswer

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self._client = self.create_client(QuestionAndAnswer, "question_and_answer")

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_req(self, str):
        req = QuestionAndAnswer.Request()
        req.question = str
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    client_node = ServiceClient()

    while True:
        str = "How do you feel?"
        response = client_node.send_req(str)
        client_node.get_logger().info("[python client] send: {0}, receive: {1}".format(
                                        str, response.answer))
        time.sleep(1)
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()