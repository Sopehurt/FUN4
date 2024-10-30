#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

STATE_SIZE = 6
MEASUREMENT_SIZE = 6
CONTROL_SIZE = 3


class ImuKalmanNode(Node):
    def __init__(self):
        super().__init__("imu_kalman_node")
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=10
        )
        self.create_subscription(
            Imu, "mpu6050_publisher", self.imu_callback, qos_profile
        )

        # Create a publisher for the "imu_kalman" topic
        self.publisher_ = self.create_publisher(Imu, "imu_kalman", qos_profile)
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize Kalman Filter variables
        self.dt = 1.0 / 76.0  # Adjust according to your IMU's sample rate
        self.state = np.zeros(
            (STATE_SIZE, 1)
        )  # [phi, theta, psi, bias_phi, bias_theta, bias_psi]

        # State covariance matrix P
        self.P = np.eye(STATE_SIZE)

        # Process noise covariance matrix Q
        process_noise_variance = 0.01  # Adjust as needed
        self.Q = process_noise_variance * np.eye(STATE_SIZE)

        # Measurement noise covariance matrix R
        measurement_noise_variance = 1.0  # Adjust as needed
        self.R = measurement_noise_variance * np.eye(MEASUREMENT_SIZE)

        # State transition matrix A
        self.A = np.eye(STATE_SIZE)
        self.A[0, 3] = -self.dt
        self.A[1, 4] = -self.dt
        self.A[2, 5] = -self.dt

        # Control matrix B
        self.B = np.zeros((STATE_SIZE, CONTROL_SIZE))
        self.B[0, 0] = self.dt
        self.B[1, 1] = self.dt
        self.B[2, 2] = self.dt

        # Observation matrix C
        self.C = np.zeros((MEASUREMENT_SIZE, STATE_SIZE))
        self.C[0, 0] = 1.0
        self.C[1, 1] = 1.0
        # The remaining rows are zeros

        self.phi_est =0.0
        self.theta_est=0.0

        self.last_msg_time = None

    def imu_callback(self, msg: Imu):

        # Get the current time
        current_time = self.get_clock().now()

        if self.last_msg_time is not None:
            # Calculate time difference in seconds
            time_diff = (current_time - self.last_msg_time).nanoseconds / 1e9
            if time_diff > 0:
                # Compute rate in Hz
                rate_hz = 1.0 / time_diff
                self.dt = rate_hz
                # self.get_logger().info(f"Subscriber rate: {rate_hz:.2f} Hz")
        else:
            self.get_logger().info("First message received.")

        # Extract sensor readings
        omega = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        ).reshape(-1, 1)

        accel = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )

        # Estimate phi and theta from accelerometer
        phi = np.arctan2(accel[1], np.sqrt(accel[0] ** 2 + accel[2] ** 2))
        theta = np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2))

        # Build measurement vector z (6x1)
        z = np.zeros((MEASUREMENT_SIZE, 1))
        z[0, 0] = phi
        z[1, 0] = theta
        # The rest are zeros

        # Compute R matrix based on current state estimates
        self.phi_est += (omega[0, 0] - self.state[3, 0]) * self.dt  # phi_est
        self.theta_est += (omega[1, 0] - self.state[4, 0]) * self.dt  # theta_est
        # phi_est = self.state[0, 0]
        # theta_est = self.state[1, 0]
        cos_phi = np.cos(self.phi_est)
        sin_phi = np.sin(self.phi_est)
        cos_theta = np.cos(self.theta_est)
        sin_theta = np.sin(self.theta_est)

        R_matrix = np.array(
            [
                [cos_theta, 0, -cos_phi * sin_theta],
                [0, 1, sin_phi],
                [sin_theta, 0, cos_phi * cos_theta],
            ]
        )

        # Compute control input u
        try:
            R_inv = np.linalg.inv(R_matrix)
            u = np.dot(R_inv, omega)
        except np.linalg.LinAlgError:
            # Handle singular matrix
            self.get_logger().warn("Singular R matrix encountered. Using omega as u.")
            u = omega

        # Kalman Filter - Prediction Step
        self.state = np.dot(self.A, self.state) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        # Kalman Filter - Update Step
        y = z - np.dot(self.C, self.state)
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        K = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        self.state = self.state + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.C), self.P)

        # Extract updated angles
        updated_phi = self.state[0, 0]
        updated_theta = self.state[1, 0]
        updated_psi = self.state[2, 0]

        # Convert updated Euler angles to quaternion
        orientation_quat = quaternion_from_euler(
            updated_phi, updated_theta, updated_psi
        )

        # Create an Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # # Set the orientation quaternion
        # imu_msg.orientation.x = orientation_quat[0]
        # imu_msg.orientation.y = orientation_quat[1]
        # imu_msg.orientation.z = orientation_quat[2]
        # imu_msg.orientation.w = orientation_quat[3]
        # Set the orientation quaternion
        imu_msg.orientation.x = 180.0*updated_phi/np.pi
        imu_msg.orientation.y = 180.0*updated_theta/np.pi
        imu_msg.orientation.z = 180.0*updated_psi/np.pi
        imu_msg.orientation.w = 0.0

        # Optionally, set angular_velocity and linear_acceleration
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration

        # Publish the message
        self.publisher_.publish(imu_msg)

        # Log the updated angles (optional)
        # self.get_logger().info(
        #     f"Published IMU Data: Roll={np.degrees(updated_phi):.2f}, "
        #     f"Pitch={np.degrees(updated_theta):.2f}, Yaw={np.degrees(updated_psi):.2f}"
        # )

        cmd_msg = Twist()
        cmd_msg.linear.x = updated_phi
        cmd_msg.angular.z = updated_theta

        self.cmd_publisher.publish(cmd_msg)

        self.last_msg_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = ImuKalmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
