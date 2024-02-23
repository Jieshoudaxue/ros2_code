#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Subscriber(Node):
    def __init__(self):
        super().__init__('test_qos_subscriber')

        qos_profile = QoSProfile(
            # reliability = QoSReliabilityPolicy.RELIABLE,
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._subscriber = self.create_subscription(String, "hello_topic", self.topic_callback, qos_profile)

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