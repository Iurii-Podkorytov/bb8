import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuCovarianceAdder(Node):
    def __init__(self):
        super().__init__('imu_covariance_adder')

        # Declare parameters for variances (stddev^2)
        # These should URDF noise parameters
        self.declare_parameter('orientation_variance', [0.001, 0.001, 0.001]) # R, P, Y
        self.declare_parameter('angular_velocity_variance', [1e-6, 1e-6, 1e-6]) # X, Y, Z
        self.declare_parameter('linear_acceleration_variance', [1e-4, 1e-4, 1e-4]) # X, Y, Z

        self.orientation_variances = self.get_parameter('orientation_variance').get_parameter_value().double_array_value
        self.angular_velocity_variances = self.get_parameter('angular_velocity_variance').get_parameter_value().double_array_value
        self.linear_acceleration_variances = self.get_parameter('linear_acceleration_variance').get_parameter_value().double_array_value

        self.imu_sub = self.create_subscription(
            Imu,
            'imu_raw',
            self.imu_callback,
            10)

        self.imu_pub = self.create_publisher(
            Imu,
            'imu_with_covariance',
            10)

        self.get_logger().info('IMU Covariance Adder node started.')
        self.get_logger().info(f" Orientation variances: {self.orientation_variances}")
        self.get_logger().info(f" Angular velocity variances: {self.angular_velocity_variances}")
        self.get_logger().info(f" Linear acceleration variances: {self.linear_acceleration_variances}")


    def imu_callback(self, msg_in: Imu):
        msg_out = msg_in # Start by copying the message

        # Populate orientation_covariance (3x3 matrix flattened, diagonal elements)
        # Indices: 0 (roll-roll), 4 (pitch-pitch), 8 (yaw-yaw)
        cov_orientation = np.zeros(9, dtype=np.float64)
        cov_orientation[0] = self.orientation_variances[0]  # Roll variance
        cov_orientation[4] = self.orientation_variances[1]  # Pitch variance
        cov_orientation[8] = self.orientation_variances[2]  # Yaw variance
        msg_out.orientation_covariance = cov_orientation.tolist()

        # Populate angular_velocity_covariance
        cov_angular_velocity = np.zeros(9, dtype=np.float64)
        cov_angular_velocity[0] = self.angular_velocity_variances[0] # X variance
        cov_angular_velocity[4] = self.angular_velocity_variances[1] # Y variance
        cov_angular_velocity[8] = self.angular_velocity_variances[2] # Z variance
        msg_out.angular_velocity_covariance = cov_angular_velocity.tolist()

        # Populate linear_acceleration_covariance
        cov_linear_acceleration = np.zeros(9, dtype=np.float64)
        cov_linear_acceleration[0] = self.linear_acceleration_variances[0] # X variance
        cov_linear_acceleration[4] = self.linear_acceleration_variances[1] # Y variance
        cov_linear_acceleration[8] = self.linear_acceleration_variances[2] # Z variance
        msg_out.linear_acceleration_covariance = cov_linear_acceleration.tolist()

        self.imu_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceAdder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()