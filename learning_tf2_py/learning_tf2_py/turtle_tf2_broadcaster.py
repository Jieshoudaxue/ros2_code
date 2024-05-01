#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped

class Helper:
    # 根据已知的数学公式，将三个欧拉角转换为四元数
    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

class TurtleTfBroadcaster(Node):
    def __init__(self, name):
        super().__init__(name)

        self._turtlename = self.declare_parameter("turtle_name", "turtle1").get_parameter_value().string_value

        self._tf_broadcaster = TransformBroadcaster(self)

        pose_topic = "/" + self._turtlename + "/pose"
        self.get_logger().info("subscribe to " + pose_topic)
        self._pose_subscription = self.create_subscription(Pose, pose_topic, self.pose_callback, 1)

        self.get_logger().info("start turtle_tf2_broadcaster node !!")

    def pose_callback(self, msg):
        tf_stamped = TransformStamped()

        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = "world"
        tf_stamped.child_frame_id = self._turtlename

        tf_stamped.transform.translation.x = msg.x
        tf_stamped.transform.translation.y = msg.y
        tf_stamped.transform.translation.z = 0.0

        tf_q = Helper.quaternion_from_euler(0, 0, msg.theta)
        tf_stamped.transform.rotation.x = tf_q[0]
        tf_stamped.transform.rotation.y = tf_q[1]
        tf_stamped.transform.rotation.z = tf_q[2]
        tf_stamped.transform.rotation.w = tf_q[3]

        self._tf_broadcaster.sendTransform(tf_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTfBroadcaster("turtle_tf2_broadcaster")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
# if __name__ == '__main__':
#     main()