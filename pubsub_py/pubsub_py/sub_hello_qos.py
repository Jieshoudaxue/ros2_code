#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration
from time import sleep

class Subscriber(Node):
    def __init__(self):
        super().__init__('test_qos_subscriber')

        qos_profile = QoSProfile(
            # RELIABLE(default), BEST_EFFORT
            reliability = QoSReliabilityPolicy.RELIABLE,
            # VOLATILE(default)
            # TRANSIENT_LOCAL(Only works when reliability is RELIABLE and needs history:KEEP_LAST and depth)
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            # history: only works when reliability is RELIABLE
            # KEEP_LAST(default), KEEP_ALL
            history = QoSHistoryPolicy.KEEP_LAST,
            # default is 10
            depth=10,
            # default is Infinite
            lifespan=Duration(seconds=5),

            # default is Infinite
            deadline=Duration(seconds=5, nanoseconds=0),
            # AUTOMATIC(default), MANUAL_BY_TOPIC
            liveliness=LivelinessPolicy.AUTOMATIC,
            # default is Infinite
            liveliness_lease_duration=Duration(seconds=5)
        )

        self._subscriber = self.create_subscription(String, "hello_topic", self.topic_callback, qos_profile)

    def topic_callback(self, msg):
        self.get_logger().info("subscribe: %s" % msg.data)
        # sleep(2)        # test qos history and depth parameters


def main(args=None):
    rclpy.init(args=args)
    
    sub_node = Subscriber()
    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()