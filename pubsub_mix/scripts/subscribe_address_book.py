#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from pubsub_mix.msg import AddressBook

class Subscriber(Node):
    def __init__(self):
        super().__init__('address_book_subscriber')
        self._subscriber = self.create_subscription(AddressBook, "address_book", self.topic_callback, 10)

    def topic_callback(self, msg):
        self.get_logger().info("subscribe in python: {0}, {1}, {2}, {3}".format(
                msg.first_name, msg.last_name, msg.phone_number, msg.phone_type))


def main(args=None):
    rclpy.init(args=args)
    
    sub_node = Subscriber()
    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()