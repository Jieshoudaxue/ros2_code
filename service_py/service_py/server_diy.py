#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from diy_interface.srv import QuestionAndAnswer

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self._service = self.create_service(QuestionAndAnswer, "question_and_answer", self.server_callback)

    def server_callback(self, request, response):
        response.answer = "feel good!"
        self.get_logger().info("[python server] receive: {0}, return: {1}".format(
                                request.question, response.answer))
        return response

def main(args=None):
    rclpy.init(args=args)

    server_node = ServiceServer()
    rclpy.spin(server_node)
    
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()