#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

class MiniParam(Node):
    def __init__(self, name):
        super().__init__(name)
        # set param description
        from rcl_interfaces.msg import ParameterDescriptor
        parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')
        # "set in python !!" will be cover by launch file: "set in launch !!"
        self.declare_parameter("demo_param", "set in python !!", parameter_descriptor)
        self._timer = self.create_timer(1, self.timer_cb)

    def timer_cb(self):
        my_param = self.get_parameter("demo_param").get_parameter_value().string_value
        self.get_logger().info("param: %s" % my_param)

        # Reset all parameters to prevent external modifications
        all_parameters = []
        demo_param = rclpy.parameter.Parameter(
            'demo_param',
            rclpy.Parameter.Type.STRING,
            'set in python !!'
        )
        all_parameters.append(demo_param)
        self.set_parameters(all_parameters)
            
            
def main(args=None):
    rclpy.init(args=args)
    node = MiniParam("test_param_py")
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()