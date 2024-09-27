#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ConfigureController  # Import the services

import sys, select, termios, tty

class TeleopModeKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_mode_key_node')

        """----------------------------------------TIMER-----------------------------------------"""
        self.freq = 100.0
        self.create_timer(1.0 / self.freq, self.timer_callback)
        

        """----------------------------------------CLIENT-----------------------------------------"""
        self.configure_controller_client = self.create_client(ConfigureController, '/mode')
        

        """-------------------------Terminal settings for keyboard input--------------------------"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("teleop_key_node has been started.")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_key(self, key):
        send_key_request = ConfigureController.Request()
        send_key_request.name = str(key)
        self.configure_controller_client.call_async(send_key_request)
        self.get_logger().info(f"Sent key {key}")

    def timer_callback(self):
        key = self.getKey()

        if key in ['0', '1', '2', '3']:
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
