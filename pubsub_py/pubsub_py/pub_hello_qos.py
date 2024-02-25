#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration

class Publisher(Node):
    def __init__(self):
        super().__init__('test_qos_publisher')

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

        self._publisher = self.create_publisher(String, "hello_topic", qos_profile)
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