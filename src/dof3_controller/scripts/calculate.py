#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from fun4_interfaces.srv import ModeControl
from geometry_msgs.msg import Twist, PoseStamped

from spatialmath import SE3
from math import pi
import roboticstoolbox as rtb
import numpy as np
from std_msgs.msg import Header, String
from math import sqrt

class CalculateNode(Node):
    def __init__(self):
        super().__init__('calculate_node')
        
        # Initialize the robot model and parameters
        self.robot = self.robot_description()

        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.eff_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.singularity_pub = self.create_publisher(String, "/singularity", 10)
        
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
            self.get_logger().info(f'{self.mode}')

        else:
            self.can_do = True
            self.tele_do = False
            x = request.ipk_x
            y = request.ipk_y
            z = request.ipk_z
            
            response.check = "Generate ramdom target success"

            random_pose_to_check = sqrt(x**2 + y**2 + (z - 0.2)**2)
            if 0.03 < random_pose_to_check < 0.53:
                self.target[0] = request.ipk_x
                self.target[1] = request.ipk_y
                self.target[2] = request.ipk_z
                response.success = True
                response.x = self.target[0]
                response.y = self.target[1]
                response.z = self.target[2]
            else:
                response.success = False
                response.x = self.target[0]
                response.y = self.target[1]
                response.z = self.target[2]
                
            
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
        if self.can_do:
            velocities_calc = None
            p_0e = self.robot.fkine(self.init_q).t[:3]
            if self.tele_do:
                if self.mode == 'TRef':
                    T_0e = self.robot.fkine(self.init_q)
                    R_0e = T_0e.R
                    velocities_calc = self.cmd_vel @ R_0e
                elif self.mode == 'BRef':
                    velocities_calc = self.cmd_vel
            else:
                error = (np.array(self.target) - np.array(p_0e)) * self.kp
                velocities_calc = error
                
            jacobian = self.robot.jacob0(self.init_q)
            jacobian_3x3 = jacobian[:3, :3]
            
            q_dot = np.linalg.pinv(jacobian_3x3) @ velocities_calc
            self.get_logger().info(f'{q_dot}')
            
            self.init_q += q_dot * (1 / self.freq)
            self.q += q_dot * (1 / self.freq)

            self.msg = JointState()
            self.msg.name = self.name
            self.msg.position = self.q.tolist()
            self.msg.header.stamp = self.get_clock().now().to_msg()
            
            if self.mode == 'TRef' or self.mode == 'BRef':
                jacobian_new = self.robot.jacob0(self.init_q)
                jacobian_new_3x3 = jacobian_new[:3, :3]
                det = np.sqrt(np.linalg.det((jacobian_new_3x3 @ jacobian_new_3x3.T)))

                if -0.001 <= det and det <= 0.001:
                    self.init_q -= q_dot * (1 / self.freq)
                    self.q -= q_dot * (1 / self.freq)
                    self.msg.position = self.q.tolist()
                    self.joint_pub.publish(self.msg)
                    
                    sing_msg = String()
                    sing_msg.data = "singularity"
                    self.singularity_pub.publish(sing_msg)
                    
                else:
                    self.joint_pub.publish(self.msg)
                    sing_msg = String()
                    sing_msg.data = "not in singularity"
                    self.singularity_pub.publish(sing_msg)
            else:  
                self.joint_pub.publish(self.msg)
                
                
            self.eff_msg = PoseStamped()
            self.eff_msg.header = Header()
            self.eff_msg.header.stamp = self.get_clock().now().to_msg()
            self.eff_msg.header.frame_id = 'link_0'
            self.eff_msg.pose.position.x = p_0e[0]
            self.eff_msg.pose.position.y = p_0e[1]
            self.eff_msg.pose.position.z = p_0e[2]
            self.eff_pub.publish(self.eff_msg)

            self.target_msg = PoseStamped()
            self.target_msg.header = Header()
            self.target_msg.header.stamp = self.get_clock().now().to_msg()
            self.target_msg.header.frame_id = 'link_0'
            self.target_msg.pose.position.x = self.target[0]
            self.target_msg.pose.position.y = self.target[1]
            self.target_msg.pose.position.z = self.target[2]
            self.target_pub.publish(self.target_msg)
            
            
            if not self.tele_do and np.linalg.norm(velocities_calc) < 0.001:
                # self.get_logger().info(f'{velocities_calc}')
                self.can_do = False
                self.msg.position = [0.0, 0.0, 0.0]
                self.joint_pub.publish(self.msg)
                mode = 'AUTO' if self.mode == 'AUTO' else 'IDLE'
                self.finish_call(mode)
                # self.get_logger().info(f'{mode}')


def main(args=None):
    rclpy.init(args=args)
    node = CalculateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
