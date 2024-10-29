#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from numpy import random
from fun4_interfaces.srv import ModeControl
from std_srvs.srv import Trigger
from math import sqrt
import yaml
import os


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__('target_pose_node')
        
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""
        
        
        """---------------------------------------CLIENT----------------------------------------"""
        
        
        """---------------------------------------SERVER----------------------------------------"""
        self.mode_server = self.create_service(ModeControl, "/mode_auto", self.mode_callback)
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.target_pose = [0.0, 0.0, 0.0]
        self.finish = False
        self.all_pose = []
        self.get_logger().info("target_pose_node has been started.")
        class NoAliasDumper(yaml.SafeDumper):
            def ignore_aliases(self, data):
                return True
        self.filename = "/home/janyawat/Documents/GitHub/FUN4/src/dof3_controller/yaml/savepath.yaml"
        with open(self.filename, 'w') as file: yaml.dump(self.all_pose, file, Dumper=NoAliasDumper, default_flow_style=False, indent=20)
    
    
    def mode_callback(self, request: ModeControl, response: ModeControl):
        self.mode = request.mode
        # self.generate_target()
        self.save_path_pose()
        response.check = "Generate ramdom target success"
        response.success = True
        response.x = self.target_pose[0]
        response.y = self.target_pose[1]
        response.z = self.target_pose[2]
    
        return response
        
    # def generate_target(self):
    #     while True:
    #         x = float(random.uniform(-0.53, 0.53))
    #         y = float(random.uniform(-0.53, 0.53))
    #         z = float(random.uniform(-0.53, 0.53))

    #         random_pose_to_check = sqrt(x**2 + y**2 + (z - 0.2)**2)

    #         if 0.03 < random_pose_to_check < 0.53:
    #             self.target_pose[0] = x
    #             self.target_pose[1] = y
    #             self.target_pose[2] = z
    #             break
            
    def save_path_pose(self):
        
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        
        with open(self.filename, "r") as file: self.all_pose = yaml.safe_load(file)
        
        if len(self.all_pose) != 0:
            self.target_pose[0] = self.all_pose[0][0]
            self.target_pose[1] = self.all_pose[0][1]
            self.target_pose[2] = self.all_pose[0][2]
            self.all_pose.pop(0)
        
        class NoAliasDumper(yaml.SafeDumper):
            def ignore_aliases(self, data):
                return True
        with open(self.filename, 'w') as file: yaml.dump(self.all_pose, file, Dumper=NoAliasDumper, default_flow_style=False, indent=20)
    
    def timer_callback(self):
        pass

        
def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
