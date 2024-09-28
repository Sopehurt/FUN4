#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi
from spatialmath import *
from geometry_msgs.msg import Twist
import numpy as np

class JointStateNode(Node):
    def __init__(self):
        super().__init__('jointstate_script_node')

        """----------------------------------- ---TIMER----------------------------------------"""
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)
        
        """-----------------------------------------PUB-----------------------------------------"""
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.create_subscription(Twist, "/q_go_state", self.angular_go_callback, 10)
        
        """----------------------------------------INIT-----------------------------------------"""
        self.q = [0.0, 0.0, 0.0]
        self.cmd_vel = [0.0, 0.0, 0.0]  # Initialize velocities
        self.name = ["joint_1", "joint_2", "joint_3"]
        
    def angular_go_callback(self, msg: Twist):
        """ Callback to update command velocities based on incoming Twist message """
        self.q[0] = msg.angular.x  # Use angular values, not linear
        self.q[1] = msg.angular.y
        self.q[2] = msg.angular.z

        self.get_logger().info(f"Received q: {self.q}")  # Log the received values

    def sim_loop(self):
        """ Main loop for simulating joint states """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(self.q)):
            msg.position.append(self.q[i])  # Populate joint positions based on cmd_vel
            msg.name.append(self.name[i])
            
        self.joint_pub.publish(msg)
        
        # Log the cmd_vel after publishing
        # self.get_logger().info(f"Published joint positions: {self.q}")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = JointStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
