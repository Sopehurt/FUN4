#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from fun4_interfaces.srv import ModeControl
from std_srvs.srv import Trigger


class ScheduleNode(Node):
    def __init__(self):
        super().__init__('schedule_node')
        
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""
        self.position_vel_publisher = self.create_publisher(Twist, "/pose_vel", 10)
        self.configulation_publisher = self.create_publisher(Twist, "/pose_position", 10)
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.create_subscription(PoseStamped, "/target", self.target_pose_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        """---------------------------------------CLIENT----------------------------------------"""
        
        """---------------------------------------SERVER----------------------------------------"""
        self.mode_server = self.create_service(ModeControl, "/mode", self.mode_callback)
        self.finish_server = self.create_service(Trigger, "/finish", self.finish_callback)
        
        """----------------------------------------INIT-----------------------------------------"""
        self.finish = False
        self.target_pose = [0, 0, 0]
        self.mode = None
        self.cmd_vel = Twist()
        self.get_logger().info("schedule_node has been started.")
        
    def cmd_vel_callback(self, msg: Twist):
        if self.mode == 'Tool Reference' or self.mode == 'Base Reference':
            self.cmd_vel.linear.x = msg.linear.x
            self.cmd_vel.linear.y = msg.linear.y
            self.cmd_vel.linear.z = msg.linear.z

    def finish_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.finish = True
        response.success = True
        response.message = "Process finished."
        self.get_logger().info(f"Please select mode")
        return response

    def target_pose_callback(self, msg: PoseStamped):
        if self.finish:
            self.target_pose[0] = msg.pose.position.x
            self.target_pose[1] = msg.pose.position.y
            self.target_pose[2] = msg.pose.position.z
            # self.get_logger().info(f"Target Position:\nx = {self.target_pose[0]}\ny = {self.target_pose[1]}\nz = {self.target_pose[2]}")
        
    def mode_callback(self, request: ModeControl, response: ModeControl):
        if self.finish:
            self.mode = request.mode

            if self.mode == 'IPK':
                self.target_pose[0] = request.ipk_x
                self.target_pose[1] = request.ipk_y
                self.target_pose[2] = request.ipk_z
                self.get_logger().info(f"Received mode: {self.mode}")
                # self.get_logger().info(f"Target Position:\nx = {self.target_pose[0]}\ny = {self.target_pose[1]}\nz = {self.target_pose[2]}")
                self.finish = False
                
            elif self.mode == 'AUTO':
                self.get_logger().info(f"Received mode: {self.mode}")
                # self.get_logger().info(f"Target Position:\nx = {self.target_pose[0]}\ny = {self.target_pose[1]}\nz = {self.target_pose[2]}")
                self.finish = False
                
            elif self.mode == 'Tool Reference' or self.mode == 'Base Reference':
                self.get_logger().info(f"Teleop by : {self.mode}")
                self.get_logger().info(f"Linear Velocity:\nx = {self.cmd_vel.linear.x}\ny = {self.cmd_vel.linear.y}\nz = {self.cmd_vel.linear.z}")
        
        return response

    def timer_callback(self):
        if not self.finish:
            if self.mode == 'IPK' or self.mode == 'AUTO':
                twist_msg = Twist()
                twist_msg.linear.x = self.target_pose[0]
                twist_msg.linear.y = self.target_pose[1]
                twist_msg.linear.z = self.target_pose[2]
                self.configulation_publisher.publish(twist_msg)
                # self.get_logger().info(f"{self.target_pose}")
                
            elif self.mode == 'Tool Reference' or self.mode == 'Base Reference':
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
