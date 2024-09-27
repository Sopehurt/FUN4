#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from numpy import random
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

import random
import math


class TeleopSubscribeNode(Node):
    def __init__(self):
        super().__init__('teleop_subscribe_node')
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        
        """---------------------------------------CLIENT-----------------------------------------"""
        
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.cmd_vel = [0,0,0]
        self.get_logger().info("teleop_subscribe_node has been started.")
        
        
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.linear.z
        
        self.get_logger().info(f"Linear Velocity:\nx = {self.cmd_vel[0]}\ny = {self.cmd_vel[1]}\nz = {self.cmd_vel[2]}")
        
    def timer_callback(self):
        pass

        

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSubscribeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
