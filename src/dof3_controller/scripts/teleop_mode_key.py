#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ConfigureController  # Import the services
from fun4_interfaces.srv import ModeControl


import sys, select, termios, tty

class TeleopModeKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_mode_key_node')

        """----------------------------------------TIMER-----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        

        """----------------------------------------CLIENT-----------------------------------------"""
        self.set_mode_client = self.create_client(ModeControl, '/mode')
        

        """-------------------------Terminal settings for keyboard input--------------------------"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        
        """----------------------------------------INIT-----------------------------------------"""
        self.cmd_vel = [0, 0, 0]
        
        
        self.get_logger().info("teleop_key_node has been started.")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_key(self, key):
        send_key_request = ModeControl.Request()
        send_key_request.mode = str(key)
        self.set_mode_client.call_async(send_key_request)
        self.get_logger().info(f"Sent key {key}")
        
    def send_cmd_vel(self, key, cmd_vel):
        send_cmd_vel_request = ModeControl.Request()
        send_cmd_vel_request.mode = str(key)
        send_cmd_vel_request.ipk_x = cmd_vel[0]
        send_cmd_vel_request.ipk_y = cmd_vel[1]
        send_cmd_vel_request.ipk_z = cmd_vel[2]
        self.set_mode_client.call_async(send_cmd_vel_request)

    def timer_callback(self):
        key = self.getKey()

        if key in ['0', '1', '2', '3']:
            if key == '1':
                try:
                    x = float(input("Please enter the value for x: "))
                    y = float(input("Please enter the value for y: "))
                    z = float(input("Please enter the value for z: "))

                    cmd_vel = [x, y, z]
                    self.send_cmd_vel(key,cmd_vel)
                except ValueError:
                    self.get_logger().error("Invalid input! Please enter valid numbers.")

            else:
                self.send_key(key)

        # Exit on Ctrl-C
        if key == '\x03':
            rclpy.shutdown()
            return

def main(args=None):
    rclpy.init(args=args)
    node = TeleopModeKeyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
