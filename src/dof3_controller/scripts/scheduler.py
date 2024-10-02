#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from fun4_interfaces.srv import ModeControl
from std_srvs.srv import Trigger


class ScheduleNode(Node):
    def __init__(self):
        super().__init__('schedule_node')
        
        """----------------------------------- ---TIMER----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        
        """-----------------------------------------PUB-----------------------------------------"""

        
        """---------------------------------------CLIENT----------------------------------------"""
        self.cal_state = self.create_client(ModeControl, '/cal_state')
        self.mode_auto_client = self.create_client(ModeControl, '/mode_auto')
        
        """---------------------------------------SERVER----------------------------------------"""
        self.mode_server = self.create_service(ModeControl, "/mode", self.server_mode_callback)

        
        """----------------------------------------INIT-----------------------------------------"""
        self.target_IPK = [0, 0, 0]
        self.target_pose = [0, 0, 0]
        self.idle = 0
        self.auto = 0
        
        self.get_logger().info("schedule_node has been started.")
        
    def finish_server_callback(self, request: ModeControl.Request, response: ModeControl.Response):
        self.finish = request.mode
        if self.finish == 'finish_go_point':
            self.get_logger().info("Finish")
            response.success = True
        return response
        
    def server_mode_callback(self, request: ModeControl.Request, response: ModeControl.Response):
        self.mode = request.mode
        self.target_IPK[0] = request.ipk_x
        self.target_IPK[1] = request.ipk_y
        self.target_IPK[2] = request.ipk_z
        
        if self.mode == 'IDLE':
            self.auto = 0
            self.idle = 1
            target_request = ModeControl.Request()
            target_request.mode = self.mode
            target_request.ipk_x = self.target_IPK[0]
            target_request.ipk_y = self.target_IPK[1]
            target_request.ipk_z = self.target_IPK[2]
            self.cal_state.call_async(target_request)

        if self.idle == 1 or self.auto == 1:

            if self.mode == 'IPK':
                self.idle = 0
                self.get_logger().info(f"mode 1")

                target_request = ModeControl.Request()
                target_request.mode = self.mode
                target_request.ipk_x = self.target_IPK[0]
                target_request.ipk_y = self.target_IPK[1]
                target_request.ipk_z = self.target_IPK[2]
                self.cal_state.call_async(target_request)

                # self.get_logger().info(f" {self.target_IPK}")
            
            elif self.mode == 'AUTO':
                self.idle = 0
                self.auto = 1
                self.get_logger().info(f"mode 3")
                send_request = ModeControl.Request()
                send_request.mode = "get_random_target"
                future = self.mode_auto_client.call_async(send_request)
                future.add_done_callback(self.callback_auto)

            elif self.mode == 'TRef':
                self.idle = 0
                target_request = ModeControl.Request()
                target_request.mode = self.mode
                self.cal_state.call_async(target_request)
                # self.get_logger().info(f" {self.mode}")
                
                

            elif self.mode == 'BRef':
                self.idle = 0
                target_request = ModeControl.Request()
                target_request.mode = self.mode
                self.cal_state.call_async(target_request)
                # self.get_logger().info(f" {self.mode}")

        
                
        return response  # Ensure that the response is returned
    
    def callback_auto(self, future:ModeControl):
        response = future.result()
        
        # self.success = response.success
        self.target_pose[0] = response.x
        self.target_pose[1] = response.y
        self.target_pose[2] = response.z
        
        # self.get_logger().info(f"{self.target_pose}")
        
        srv = ModeControl.Request()
        srv.mode = self.mode
        srv.ipk_x = self.target_pose[0]
        srv.ipk_y = self.target_pose[1]
        srv.ipk_z = self.target_pose[2]
        
        self.cal_state.call_async(srv)
        # self.get_logger().info(f"{response.success}")


    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
