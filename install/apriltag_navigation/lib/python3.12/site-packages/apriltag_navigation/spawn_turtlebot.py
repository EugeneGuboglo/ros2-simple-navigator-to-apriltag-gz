#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleTurtleBot(Node):
    def __init__(self):
        super().__init__('simple_turtlebot')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Simple TurtleBot node started')
    
    def move_forward(self, speed=0.2):
        msg = Twist()
        msg.linear.x = speed
        self.cmd_pub.publish(msg)
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleTurtleBot()
    
    # Простой тест движения
    try:
        node.get_logger().info('Moving forward for 3 seconds...')
        node.move_forward()
        time.sleep(3)
        node.stop()
        node.get_logger().info('Stopped')
    except KeyboardInterrupt:
        node.stop()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
