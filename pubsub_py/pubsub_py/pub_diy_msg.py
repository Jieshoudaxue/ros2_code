#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from diy_interface.msg import Student
from diy_interface.msg import Sphere

class Publisher(Node):
    def __init__(self):
        super().__init__('test_pub_diy_msg')
        self._pub_student = self.create_publisher(Student, "student_topic", 10)
        self._pub_sphere = self.create_publisher(Sphere, "Sphere_topic", 10)
        self._timer = self.create_timer(0.5, self.timer_callback)
        self._i = 0

        self._students = [
            ("yi", 32),
            ("miao", 18),
            ("bao", 3)
        ]
        self._spheres = [
            ((1.0, 2.0, 3.0), 4.0),
            ((1.1, 2.1, 3.1), 4.1)
        ]

    def timer_callback(self):
        stu_msg = Student()
        stu_msg.name = self._students[self._i % len(self._students)][0]
        stu_msg.age = self._students[self._i % len(self._students)][1]
        self.get_logger().info("Publishing student in python: {0}, {1}".format(stu_msg.name, stu_msg.age))
        self._pub_student.publish(stu_msg)

        sph_msg = Sphere()
        sph_msg.center.x = self._spheres[self._i % len(self._spheres)][0][0]
        sph_msg.center.y = self._spheres[self._i % len(self._spheres)][0][1]
        sph_msg.center.z = self._spheres[self._i % len(self._spheres)][0][2]
        sph_msg.radius = self._spheres[self._i % len(self._spheres)][1]
        self.get_logger().info("Publishing sphere in python: ({0}, {1}, {2}), {3}".format(sph_msg.center.x, sph_msg.center.y, sph_msg.center.z, sph_msg.radius))
        self._pub_sphere.publish(sph_msg)

        self._i += 1


def main(args=None):
    rclpy.init(args=args)

    pub_node = Publisher()
    rclpy.spin(pub_node)
    
    pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()