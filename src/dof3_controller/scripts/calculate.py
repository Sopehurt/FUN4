#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from spatialmath import SE3
from math import pi, atan2, sqrt

from geometry_msgs.msg import PoseStamped, Twist
from fun4_interfaces.srv import ModeControl
from std_srvs.srv import Trigger

import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
import math
import time


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__('target_pose_node')
        
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.create_subscription(Twist, "/pose_position", self.pose_position_callback, 10)
        self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)
        
        """----------------------------------------INIT-----------------------------------------"""
        self.pose_position = [0, 0, 0]
        self.q_setpoint = [0, 0, 0]
        self.q_current = [0, 0, 0]
        self.q_go = [0, 0, 0]

        # PID parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.0  # Integral gain (can tune later)
        self.kd = 0.0  # Derivative gain (can tune later)

        # For PID control
        self.error_last = [0, 0, 0]  # To store previous error for derivative term
        self.error_sum = [0, 0, 0]  # To accumulate errors for integral term
        self.last_time = time.time()

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """
        Convert a quaternion into a rotation matrix.
        """
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        return R

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            self.process_transform(transform)
    
    def process_transform(self, transform: TransformStamped):
        frame_id = transform.header.frame_id
        child_frame_id = transform.child_frame_id
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        x = translation.x
        y = translation.y
        z = translation.z
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w
        
        # Convert quaternion to rotation matrix manually
        R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
        
        # Construct SE3 transformation matrix from the rotation matrix and translation
        P_setpoint = SE3.Rt(R, [x, y, z])  # R is rotation matrix, [x, y, z] is translation
        
        # Call inverse kinematics to get current joint angles
        q_current, *_ = self.robot.ikine_LM(P_setpoint, mask=[1, 1, 1, 0, 0, 0])
        
        self.q_current = q_current  # Update current joint angles
        
    def pose_position_callback(self, msg: Twist):
        self.pose_position[0] = msg.linear.x
        self.pose_position[1] = msg.linear.y
        self.pose_position[2] = msg.linear.z
        self.inverse_kinematic(self.pose_position[0], self.pose_position[1], self.pose_position[2])
        
    def inverse_kinematic(self, x, y, z):
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
        
        r_go = sqrt(x**2 + y**2 + (z-0.2)**2)
        if 0.53 > r_go > 0.3:
            P_setpoint = SE3(x, y, z)
            q_, *pointer = robot.ikine_LM(P_setpoint, mask=[1, 1, 1, 0, 0, 0], q0=[0, 0, 0])
            self.q_setpoint = [q_[0], q_[1], q_[2]]  # Update desired joint angles

    def calculate_q_go(self):
        current_time = time.time()
        dt = current_time - self.last_time

        q_go = [0, 0, 0]  # Initialize control signal list
        for i in range(3):
            # Proportional error
            error = self.q_setpoint[i] - self.q_current[i]
            
            # Integral term (accumulating error over time)
            self.error_sum[i] += error * dt
            
            # Derivative term (rate of change of error)
            d_error = (error - self.error_last[i]) / dt if dt > 0 else 0.0

            # PID control signal
            q_go[i] = (self.kp * error) + (self.ki * self.error_sum[i]) + (self.kd * d_error)
            
            # Store current error for next derivative calculation
            self.error_last[i] = error

        self.q_go = q_go  # Update control signal
        self.last_time = current_time  # Update time for next cycle
        
        self.get_logger().info(f"q_go: {q_go}")
    
    def timer_callback(self):
        # Calculate control signal using PID
        self.calculate_q_go()
        

def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
