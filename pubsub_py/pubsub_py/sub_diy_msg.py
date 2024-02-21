#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from diy_interface.msg import Student
from diy_interface.msg import Sphere

class Subscriber(Node):
    def __init__(self):
        super().__init__('test_sub_diy_msg')
        self._sub_student = self.create_subscription(Student, "student_topic", self.student_topic_callback, 10)
        self._sub_sphere = self.create_subscription(Sphere, "sphere_topic", self.sphere_topic_callback, 10)

    def student_topic_callback(self, msg):
        self.get_logger().info("i received student in python: {0}, {1}".format(msg.name, msg.age))

    def sphere_topic_callback(self, msg):
        self.get_logger().info("i received sphere in python: ({0}, {1}, {2}), {3}".format(msg.center.x, msg.center.y, msg.center.z, msg.radius))


def main(args=None):
    rclpy.init(args=args)
    
    sub_node = Subscriber()
    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()