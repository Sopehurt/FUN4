#!/usr/bin/python3
import roboticstoolbox as rtb
import numpy as np


from sensor_msgs.msg import JointState
from spatialmath import SE3
from math import pi
from math import radians
import rclpy
from rclpy.node import Node
from scipy.optimize import minimize
# from geometry_msgs.msg import TransformStamped, PoseStamped
from fun4_interfaces.srv import ChangeMode
# from tf_transformations import euler_from_quaternion

class PIDController:
    def __init__(self, kp):
        self.kp = kp
        self.prev_error = 0

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        output = self.kp * error
        return output

class InversePoseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_pose_kinematics')
        self.get_logger().info('inverse pose kinematics node has start')
        # Pub
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update_joint_states)
        # Sub
        # self.target_sub = self.create_subscription(PoseStamped, '/target', self.callback_target, 10)

        # Service Server (รับ mode เข้ามา)
        self.take_mode = self.create_service(ChangeMode, '/mode_pose', self.callback_user)

        # Innitial
        self.mode = 0
        self.x ,self.y, self.z = 0.0, 0.0, 0.0
        self.finish = None
        self.r_min = 0.03
        self.r_max = 0.535
        self.answer = []
        self.current_joint_positions = [0.0, 0.0, 0.0]
        self.target_joint_positions = [0.0, 0.0, 0.0]

        # Initialize PID controllers with kp only
        self.pid_controllers = [PIDController(1.0),  # For joint 1
                                PIDController(1.0),  # For joint 2
                                PIDController(1.0)]  # For joint 3
        
        # Indicate whether a new target position is set
        self.new_target = False

        # Tolerance for considering joint to have reached the target
        self.tolerance = 0.01  # Adjust this tolerance based on your system's requirements

    def callback_user(self,request:ChangeMode.Request, response:ChangeMode.Response): # รับ
        mode = request.mode
        x = request.pose.x
        y = request.pose.y
        z = request.pose.z

        q_sol = self.inverse_kinematic(x, y, z, mode)
        
        response.success = self.finish
        response.config = q_sol

        return response

    def inverse_kinematic(self, x, y, z, mode):
        distance_squared = x**2 + y**2 + (z-0.2)**2
        if mode == 1 and self.r_min**2 <= distance_squared <= self.r_max**2:

            robot = rtb.DHRobot(
                [
                    rtb.RevoluteMDH(alpha = 0.0,a = 0.0,d = 0.2,offset = 0.0),
                    rtb.RevoluteMDH(alpha = pi/2,a = 0.0,d = 0.02,offset = 0.0),
                    rtb.RevoluteMDH(alpha = 0,a = 0.25,d = 0.0,offset = 0.0),

                ],tool = SE3.Tx(0.28),
                name = "RRR_Robot"
            )

            # Desired end effector pose
            T_Position = SE3(x, y, z)
            q_sol_ik_LM, *_ = robot.ikine_LM(T_Position,mask=[1,1,1,0,0,0],q0=[0,0,0])
            self.target_joint_positions = [q_sol_ik_LM[0], q_sol_ik_LM[1], q_sol_ik_LM[2]]
            self.finish = True
            self.get_logger().info('call finish')
            return self.target_joint_positions
        else:
            self.finish = False
            self.get_logger().info('call not finish')
            return None
        
    def update_joint_states(self):
        """Smoothly update joint positions using proportional control"""
        joint_msg = JointState()
        joint_msg.name = ['joint_1', 'joint_2', 'joint_3']
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        # PID (Proportional only) control to move towards target joint positions
        smoothed_positions = []
        for i in range(3):
            current_pos = self.current_joint_positions[i]
            target_pos = self.target_joint_positions[i]
            smoothed_pos = self.pid_controllers[i].compute(target_pos, current_pos)
            new_position = current_pos + smoothed_pos * self.dt
            smoothed_positions.append(new_position)

        # Update current positions to smoothed positions for next iteration
        self.current_joint_positions = smoothed_positions
        joint_msg.position = smoothed_positions

        # Publish the updated joint states
        self.joint_state_pub.publish(joint_msg)
    


def main(args=None):
    rclpy.init(args=args)
    node = InversePoseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
