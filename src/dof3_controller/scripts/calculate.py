#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from fun4_interfaces.srv import ModeControl
from geometry_msgs.msg import Twist

from spatialmath import SE3
from math import pi, sqrt
import roboticstoolbox as rtb
import numpy as np
import time


class CalculateNode(Node):
    def __init__(self):
        super().__init__('calculate_node')
        
        # Initialize the robot model and parameters
        self.robot = self.robot_description()

        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        
        """---------------------------------------SERVER----------------------------------------"""
        self.server_cal_state = self.create_service(ModeControl, "/cal_state", self.server_cal_state_callback)
        
        
        """---------------------------------------CLIENT----------------------------------------"""
        self.mode_client = self.create_client(ModeControl, '/mode')
        
        
        self.kp = 1
        self.init_q = np.array([0.0, 0.0, 0.0])
        self.q = np.array([0.0, 0.0, 0.0])
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.get_logger().info("target_pose_node has been started.")
        self.can_do = False
        self.tele_do = False
        self.target = np.array([0.0, 0.0, 0.0])
        self.mode = ''
        self.cmd_vel = np.array([0.0, 0.0, 0.0])

        
    def server_cal_state_callback(self, request: ModeControl.Request, response: ModeControl.Response):
        self.mode = request.mode
        if self.mode == 'IDLE':
            self.can_do = False
            self.tele_do = False
            self.get_logger().info('IDLE')
        
        elif self.mode == 'TRef' or self.mode == 'BRef':
            self.tele_do = True
            self.can_do = True
            self.get_logger().info('Teleop mode')

        else:
            self.can_do = True
            self.target[0] = request.ipk_x
            self.target[1] = request.ipk_y
            self.target[2] = request.ipk_z
            
        return response
    
    def robot_description(self):
        """ Define the robot description using DH parameters """
        L1 = 0.200
        L2 = 0.120
        L3 = 0.100
        L4 = 0.250
        L5 = 0.280

        robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0, a=0.0, d=L1, offset=0.0),
                rtb.RevoluteMDH(alpha=-pi/2, a=0.0, d=-L2, offset=-pi/2),
                rtb.RevoluteMDH(alpha=0.0, a=L4, d=L3, offset=0.0),
            ], 
            tool=SE3([
                [0, 0, 1, L5],
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]]),
            name="FUN4_Robot"
        )
        
        return robot
    
    
    def finish_call(self, mode):
        send_request = ModeControl.Request()
        send_request.mode = str(mode)
        self.mode_client.call_async(send_request)
    
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.linear.z
        
    def timer_callback(self):
        self.get_logger().info(f" {self.cmd_vel}")
        if self.can_do:
            if self.tele_do:
                
                if self.mode == 'TRef':
                    jacobian = self.robot.jacobe(self.init_q)
                elif self.mode == 'BRef':
                    jacobian = self.robot.jacob0(self.init_q)
                    
                jacobian_3x3 = jacobian[:3, :3]    
                q_dot = np.linalg.pinv(jacobian_3x3).dot(self.cmd_vel)
                
            else:  # Teleoperation is False   
                p_0e = self.robot.fkine(self.init_q).t[:3]
                error = (np.array(self.target) - np.array(p_0e)) * self.kp
                matrix_3x6 = self.robot.jacob0(self.init_q)
                matrix_3x3 = matrix_3x6[:3, :3]
                q_dot = np.linalg.pinv(matrix_3x3).dot(error)
                    
            self.init_q += q_dot * (1 / self.freq)

            self.msg = JointState()
            self.msg.name = self.name
            self.msg.position = self.q.tolist()  # Ensure it's a list for publishing
            
            self.q += q_dot * (1 / self.freq)
                
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.msg)
            self.get_logger().info('go')
            
            if np.linalg.norm(error) < 0.001:
                self.get_logger().info(f'{error}')
                self.can_do = False
                self.msg.position = [0.0, 0.0, 0.0]
                self.joint_pub.publish(self.msg)
                    
                if self.mode == 'AUTO':
                    mode = 'AUTO'
                else:
                    mode = 'IDLE'
                
                self.finish_call(mode)
                self.get_logger().info(f'{mode}')
                    

def main(args=None):
    rclpy.init(args=args)
    node = CalculateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
