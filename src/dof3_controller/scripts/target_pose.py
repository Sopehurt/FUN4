#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from numpy import random
from fun4_interfaces.srv import ModeControl
from std_srvs.srv import Trigger


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__('target_pose_node')
        
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""
        self.target_publisher = self.create_publisher(PoseStamped, "/target", 10)
        
        """---------------------------------------SERVER----------------------------------------"""
        self.mode_server = self.create_service(ModeControl, "/mode", self.mode_callback)
        self.finish_server = self.create_service(Trigger, "/finish", self.finish_callback)
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.target_pose = [0.0, 0.0, 0.0]
        self.finish = False
        self.get_logger().info("target_pose_node has been started.")
        
    
    
    def finish_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.finish = True
        response.success = True
        response.message = "Process finished."
        self.get_logger().info(f"Please select mode")
        return response
    
    
    def mode_callback(self, request: ModeControl, response: ModeControl):
        self.mode = request.mode
        
        if self.mode == 'AUTO':
            self.generate_target()
            
        return response
        
    def generate_target(self):
        while True:
            x = float(random.uniform(-0.53, 0.53))  # Ensure x, y, z are floats
            y = float(random.uniform(-0.53, 0.53))
            z = float(random.uniform(-0.53, 0.53))

            random_pose_to_check = x**2 + y**2 + (z - 0.2)**2

            if 0.03**2 < random_pose_to_check < 0.53**2:
                self.target_pose[0] = x
                self.target_pose[1] = y
                self.target_pose[2] = z
                break
    
    def timer_callback(self):
        if self.finish == True:
            self.generate_target()
            self.finish = False
            
        msg = PoseStamped()
        msg.pose.position.x = float(self.target_pose[0])
        msg.pose.position.y = float(self.target_pose[1])
        msg.pose.position.z = float(self.target_pose[2])
        self.target_publisher.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
