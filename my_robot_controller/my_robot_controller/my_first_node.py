#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("hello "+ str(self.counter))
        self.counter +=1

def main(args=None):
    rclpy.init(args=args) 
    #viet code o day
    node =MyNode()
    rclpy.spin(node) #be kept alive until being killed (control C)

    #
    rclpy.shutdown()

if __name__ == '__main__':
    main()


