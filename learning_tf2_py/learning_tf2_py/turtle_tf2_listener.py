#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

class TurtleTfListener(Node):
    def __init__(self, name):
        super().__init__(name)


        self._target_frame = self.declare_parameter("target_frame", "turtle1").get_parameter_value().string_value        

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._spawn_client = self.create_client(Spawn, "spawn")
        self._cmd_publisher = self.create_publisher(Twist, "turtle2/cmd_vel", 10)

        self._turtle_spawning_service_ready = False
        self._turtle_spawned = False

        self._timer = self.create_timer(1, self.timer_callback)

        self.get_logger().info("start turtle_tf2_listener node !!")

    def timer_callback(self):
        if False == self._turtle_spawning_service_ready:
            if False == self._spawn_client.service_is_ready():
                self.get_logger().info('Waiting for the spawn service to be ready')
                return

            self._turtle_spawning_service_ready = True

        if False == self._turtle_spawned:
            request = Spawn.Request()
            request.x = 5.0
            request.y = 5.0
            request.theta = 0.0
            request.name = "turtle2"

            self._spawn_client.call_async(request)
            self._turtle_spawned = True
            return

        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                "turtle2",
                self._target_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform turtle2 to {self._target_frame}: {ex}')
            return

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5 * math.sqrt(tf_stamped.transform.translation.x ** 2 + tf_stamped.transform.translation.y ** 2)
        cmd_vel.angular.z = 1.0 * math.atan2(tf_stamped.transform.translation.y, tf_stamped.transform.translation.x)

        self.get_logger().info("send cmd_vel to turtle2: linear.x: %f, angular.z: %f" % (cmd_vel.linear.x, cmd_vel.angular.z))
        self._cmd_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    node = TurtleTfListener("turtle_tf2_listener")
    rclpy.spin(node)

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()