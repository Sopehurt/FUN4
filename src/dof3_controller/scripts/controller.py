#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from numpy import random
from std_srvs.srv import Trigger
import random
import math
from controller_manager_msgs.srv import ConfigureController 


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        
        """-----------------------------------------SUB-----------------------------------------"""
        
        
        
        """---------------------------------------client----------------------------------------"""
        self.finish_client = self.create_client(Trigger,"/finish")
        
        
        """---------------------------------------server----------------------------------------"""
        
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.target_pose = [0 ,0 ,0]
        self.finish = False
        self.mode = 0
        self.get_logger().info("controller_node has been started.")
        
    def timer_callback(self):
        pass
        

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
