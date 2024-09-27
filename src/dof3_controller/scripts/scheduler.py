#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from numpy import random
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import ConfigureController 
from sensor_msgs.msg import JointState

import random
import math


class ScheduleNode(Node):
    def __init__(self):
        super().__init__('schedule_node')
        
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        
        """-----------------------------------------PUB-----------------------------------------"""
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.create_subscription(PoseStamped, "/target", self.target_pose_callback, 10)
        
        
        """---------------------------------------client----------------------------------------"""
        
        
        
        """---------------------------------------SERVER----------------------------------------"""
        self.mode_server = self.create_service(ConfigureController,"/mode",self.mode_callback)
        
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.target_pose = [0 ,0 ,0]
        self.mode = 0
        self.get_logger().info("schedule_node has been started.")
        
    def target_pose_callback(self, msg: PoseStamped):
        self.target_pose[0] = msg.pose.position.x
        self.target_pose[1] = msg.pose.position.y
        self.target_pose[2] = msg.pose.position.z
        
        self.get_logger().info(f"Target Pose: x={self.target_pose[0]}, y={self.target_pose[1]}, z={self.target_pose[2]}")
        
    def mode_callback(self, request :ConfigureController, response :ConfigureController):
        self.mode = request.name
        self.get_logger().info(f"Received mode: {self.mode}")
        return response

    def timer_callback(self):
        if self.mode == '1':
            pass
        elif self.mode == '2':
            pass
        elif self.mode == '3':
            pass
        else:
            pass
            

        

def main(args=None):
    rclpy.init(args=args)
    node = ScheduleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
