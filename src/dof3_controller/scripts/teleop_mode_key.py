#!/usr/bin/python3

import rclpy
from rclpy.node import Node
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
        self.target = [0, 0, 0]
        
        
        self.get_logger().info("teleop_key_node has been started.")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def send_mode_IPK(self, mode, target):
        send_request = ModeControl.Request()
        send_request.mode = str(mode)
        send_request.ipk_x = target[0]
        send_request.ipk_y = target[1]
        send_request.ipk_z = target[2]
        self.set_mode_client.call_async(send_request)
        
    def send_mode_Teleop(self, reference):
        send_request = ModeControl.Request()
        if reference == 1:
            send_request.mode = 'TRef'

        elif reference == 2:
            send_request.mode = 'BRef'
        
        self.set_mode_client.call_async(send_request)

        
    def send_mode_Auto(self, mode):
        send_request = ModeControl.Request()
        send_request.mode = str(mode)
        self.set_mode_client.call_async(send_request)

    def timer_callback(self):
        key = self.getKey()

        if key == '1':
            try:
                x = float(input("Please enter the value for x: "))
                y = float(input("Please enter the value for y: "))
                z = float(input("Please enter the value for z: "))
                target = [x, y, z]
                mode = 'IPK'
                self.send_mode_IPK(mode, target)
                
            except ValueError:
                self.get_logger().error("Invalid input! Please enter valid numbers.")

        elif key == '2':
            try:
                reference = int(input("Select Reference Frame: "))
                mode = 'TELEOP'
                self.send_mode_Teleop(reference)
                
            except ValueError:
                self.get_logger().error("Invalid input! Please enter 1 or 2.")
                
        elif key == '3':
            mode = 'AUTO'
            self.send_mode_Auto(mode)
            
        elif key == '0':
            mode = 'IDLE'
            self.send_mode_Auto(mode)

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
