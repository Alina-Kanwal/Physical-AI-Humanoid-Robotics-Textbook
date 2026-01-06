#!/usr/bin/env python3

"""
Example showing how to use parameters in ROS 2
"""

import rclpy
from rclpy.node import Node


class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_name', 'my_robot')

        # Access parameter values
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value

        # Create a timer that uses the parameter
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info(f'Robot {self.robot_name} at {self.frequency} Hz - count: {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()