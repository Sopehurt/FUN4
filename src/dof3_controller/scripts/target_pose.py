#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from numpy import random
from fun4_interfaces.srv import ModeControl
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
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
        # self.set_mode_client = self.create_client(ModeControl, '/mode')
        self.set_mode_client = self.create_client(SetBool, '/Mode')
        
        """---------------------------------------SERVER----------------------------------------"""
        self.mode_server = self.create_service(ModeControl, "/mode_auto", self.mode_callback)
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.target_pose = [0.0, 0.0, 0.0]
        self.finish = False
        self.all_pose = []
        self.end = 0
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
        

        if self.end == 1:
            send_request = SetBool.Request()
            send_request.data = True
            self.set_mode_client.call_async(send_request)

        return response
        
            
    def save_path_pose(self):
        
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        
        with open(self.filename, "r") as file: self.all_pose = yaml.safe_load(file)
        self.end = len(self.all_pose) + 1
        
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