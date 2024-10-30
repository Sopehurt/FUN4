#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python import get_package_share_directory
from sensor_msgs.msg import Imu

import os
import numpy as np
import yaml


class ImuCalibrationNode(Node):
    def __init__(self):
        super().__init__('imu_calib_node')
        self.declare_parameter('file', 'imu_calibration.yaml')
        pkg_name = 'imu_calibration'

        qos_progile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.create_subscription(Imu, "mpu6050_publisher", self.imu_callback, qos_progile)
        
        imu_calib_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = imu_calib_pkg_share_path.split('install')

        file = self.get_parameter('file').value
        self.imu_calib_path = os.path.join(ws_path, 'src', pkg_name, 'config', file)

        print(f"calibration file is saved at: {self.imu_calib_path}")

        self.n = 0
        self.n_max = 10000
        self.acc_list = []
        self.gyro_list = []

    
    def save_calibration(self, mean, cov, name : str):

        with open(self.imu_calib_path, 'r') as file:
            value = yaml.safe_load(file) or {}
        
        mean_list = mean.tolist()
        cov_list = cov.tolist()

        value[f"{name} offset"] = mean_list
        value[f"{name} covariance"] = cov_list

        with open(self.imu_calib_path, 'w') as file:
            yaml.dump(value, file)

    def imu_callback(self, msg : Imu):
        if self.n < self.n_max:
            self.n = self.n + 1
            self.acc_list.append([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z - 9.80665
            ])

            self.gyro_list.append([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

            print("collect data: " + str(self.n))

        else:
            acc_array = np.array(self.acc_list)
            acc_offset = np.mean(acc_array, 0)
            acc_cov = np.absolute(np.cov(acc_array.T))

            gyro_array = np.array(self.gyro_list)
            gyro_offset = np.mean(gyro_array, 0)
            gyro_cov = np.absolute(np.cov(gyro_array.T))

            self.save_calibration(acc_offset, acc_cov, 'acc')
            self.save_calibration(gyro_offset, gyro_cov, 'gyro')

            print('=======================')
            print(acc_offset)
            print(acc_cov)
            print('=======================')
            print(gyro_offset)
            print(gyro_cov)
            exit()
                    


def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
