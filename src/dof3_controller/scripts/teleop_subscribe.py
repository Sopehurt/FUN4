#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from fun4_interfaces.srv import ModeControl



class TeleopSubscribeNode(Node):
    def __init__(self):
        super().__init__('teleop_subscribe_node')
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        
        """---------------------------------------CLIENT-----------------------------------------"""
        
        """---------------------------------------SERVER----------------------------------------"""
        self.mode_server = self.create_service(ModeControl, "/mode", self.mode_callback)
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.mode = None
        self.cmd_vel = Twist()
        self.get_logger().info("teleop_subscribe_node has been started.")
        
    def mode_callback(self, request: ModeControl, response: ModeControl):
        self.mode = request.mode
        self.cmd_vel_publisher.publish(self.cmd_vel)
        return response
            
    def cmd_vel_callback(self, msg: Twist):
        if self.mode == 'Tool Reference' or self.mode == 'Base Reference':
            self.cmd_vel.linear.x = msg.linear.x
            self.cmd_vel.linear.y = msg.linear.y
            self.cmd_vel.linear.z = msg.linear.z
        
    def timer_callback(self):
        pass

        
def main(args=None):
    rclpy.init(args=args)
    node = TeleopSubscribeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
