#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

class MiniParam(Node):
    def __init__(self, name):
        super().__init__(name)
        # set param description
        parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')

        self.declare_parameter("demo_param", "set in python !!", parameter_descriptor)

        # Monitor updates to 'demo_param'
        self.add_on_set_parameters_callback(self.demo_param_change_callback)

    def demo_param_change_callback(self, params):
        result = SetParametersResult()  # Create a result instance
        for param in params:
            if param.name == 'demo_param':
                self.get_logger().info(f'received an update to : {param.name}, type is {type(param.value).__name__}, value is {param.value}')
                result.successful = True
        return result
            
def main(args=None):
    rclpy.init(args=args)
    node = MiniParam("param_monitor_py")
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()