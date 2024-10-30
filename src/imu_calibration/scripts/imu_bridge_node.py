#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from imu_interfaces.srv import ImuCalibration
import yaml
import os

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        self.declare_parameter('file', 'imu_calibration.yaml')
        pkg_name = 'imu_calibration'

        imu_calib_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = imu_calib_pkg_share_path.split('install')

        file = self.get_parameter('file').value
        imu_calib_path = os.path.join(ws_path, 'src', pkg_name, 'config', file)

        calib_client = self.create_client(ImuCalibration, 'mpu6050_calibration')
        while not calib_client.wait_for_service(timeout_sec=1.0):
            print('waiting...')

        request = ImuCalibration.Request()

        with open(imu_calib_path, 'r') as file:
            value = yaml.safe_load(file)

        request.imu_calib.linear_acceleration_covariance = [
            item for sublist in value['acc covariance'] for item in sublist
        ]
        request.imu_calib.linear_acceleration.x = value['acc offset'][0]
        request.imu_calib.linear_acceleration.y = value['acc offset'][1]
        request.imu_calib.linear_acceleration.z = value['acc offset'][2]

        request.imu_calib.angular_velocity_covariance = [
            item for sublist in value['gyro covariance'] for item in sublist
        ]
        request.imu_calib.angular_velocity.x = value['gyro offset'][0]
        request.imu_calib.angular_velocity.y = value['gyro offset'][1]
        request.imu_calib.angular_velocity.z = value['gyro offset'][2]

        while True:
            future = calib_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            result = ImuCalibration.Response()
            result = future.result()

            print(result.success)

            if result.success == True:
                exit()
    

def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
