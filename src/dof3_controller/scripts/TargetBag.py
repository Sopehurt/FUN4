#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from fun4_interfaces.srv import ModeControl
import yaml
import os


class TargetBagNode(Node):
    def __init__(self):
        super().__init__('target_bag_node')
        
        """----------------------------------------TIMER----------------------------------------"""
        self.freq = 1000.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""
        
        
        
        """-----------------------------------------SUB-----------------------------------------"""
        self.end_effector_sub = self.create_subscription(PoseStamped, '/end_effector', self.end_effector_callback, 10)
        
        
        """---------------------------------------CLIENT----------------------------------------"""
        self.set_mode_client = self.create_client(ModeControl, '/mode')
        
        
        """---------------------------------------SERVER----------------------------------------"""
        
        self.save_path_server = self.create_service(SetBool, "/SavePath", self.save_path_callback)
        self.ref_server = self.create_service(SetBool, "/Ref", self.ref_callback)
        self.mode_server = self.create_service(SetBool, "/Mode", self.mode_callback)
        
        """----------------------------------------INIT-----------------------------------------"""
        self.save_path = False
        self.ref = False
        self.mode = False
        self.reference = 1
        self.mode = 0
        self.end_effector = [0.0, 0.0, 0.0]
        self.list_point = []
        
    def end_effector_callback(self, msg: PoseStamped):
        self.end_effector[0] = msg.pose.position.x
        self.end_effector[1] = msg.pose.position.y
        self.end_effector[2] = msg.pose.position.z
    
    def save_path_callback(self, request: SetBool, response: SetBool):
        # Set response values as there is no request data in Trigger service
        self.save_path = request.data
        self.get_logger().info("save path")
        
        self.list_point.append(self.end_effector[:])
        print(self.list_point)
        
        filename = "/home/janyawat/Documents/GitHub/FUN4/src/dof3_controller/yaml/savepath.yaml"
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        class NoAliasDumper(yaml.SafeDumper):
            def ignore_aliases(self, data):
                return True
        with open(filename, 'w') as file: yaml.dump(self.list_point, file, Dumper=NoAliasDumper, default_flow_style=False, indent=20)
        
        return response
    
    def ref_callback(self, request: SetBool, response: SetBool):
        # Set response values as there is no request data in Trigger service
        self.ref = request.data
        self.get_logger().info("change Referebce")
        
        send_request = ModeControl.Request()
        
        if self.mode == 0:
            self.mode = 1
            send_request.mode = 'AUTO'
        else:
            self.mode =0
            send_request.mode = 'IDLE'
            
        
        self.set_mode_client.call_async(send_request)
        
        return response
    
    def mode_callback(self, request: SetBool, response: SetBool):
        # Set response values as there is no request data in Trigger service
        self.get_logger().info("change Mode")
        send_request = ModeControl.Request()
        if self.mode == 0:
            send_request.mode = 'IDLE'
            self.set_mode_client.call_async(send_request)
            
            if self.reference == 1:
                self.reference = 2
                send_request.mode = 'BRef'
            else:
                self.reference = 1  
                send_request.mode = 'TRef'
            
            self.set_mode_client.call_async(send_request)
        return response
    
    
    
    def timer_callback(self):
        pass

        
def main(args=None):
    rclpy.init(args=args)
    node = TargetBagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
