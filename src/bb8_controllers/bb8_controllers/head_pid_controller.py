#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import tf_transformations
import math

class HeadStabilizer(Node):
    def __init__(self):
        super().__init__('head_stabilizer_node')

        # --- Parameters ---
        self.declare_parameter('imu_topic', '/imu_base')
        self.declare_parameter('command_topic', '/head_controller/commands')
        self.declare_parameter('joint_names', ['head_x_joint', 'head_y_joint'])

        self.declare_parameter('limit_x', -math.pi / 9.0)
        self.declare_parameter('limit_y', -math.pi / 9.0)

        # PID gains for X (roll) and Y (pitch)
        self.declare_parameter('Kp_x', 1.0)
        self.declare_parameter('Ki_x', 0.0)
        self.declare_parameter('Kd_x', 0.0)
        
        self.declare_parameter('Kp_y', 1.0)
        self.declare_parameter('Ki_y', 0.0)
        self.declare_parameter('Kd_y', 0.0)
        
        self.declare_parameter('integral_max_x', 0.1)
        self.declare_parameter('integral_max_y', 0.1)

        # Retrieve parameters
        imu_topic = self.get_parameter('imu_topic').value
        command_topic = self.get_parameter('command_topic').value
        self._joint_names = self.get_parameter('joint_names').value
        self._limit_x = self.get_parameter('limit_x').value
        self._limit_y = self.get_parameter('limit_y').value

        self._Kp_x = self.get_parameter('Kp_x').value
        self._Ki_x = self.get_parameter('Ki_x').value
        self._Kd_x = self.get_parameter('Kd_x').value
        self._Kp_y = self.get_parameter('Kp_y').value
        self._Ki_y = self.get_parameter('Ki_y').value
        self._Kd_y = self.get_parameter('Kd_y').value
        self._integral_max_x = self.get_parameter('integral_max_x').value
        self._integral_max_y = self.get_parameter('integral_max_y').value

        if len(self._joint_names) != 2:
            self.get_logger().error("Expected 2 joint names in 'joint_names' parameter.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Subscribing to IMU on: {imu_topic}")
        self.get_logger().info(f"Publishing commands to: {command_topic}")
        self.get_logger().info(f"Controlling joints: {self._joint_names}")

        # PID variables for X (roll) and Y (pitch)
        self._prev_error_x = 0.0
        self._integral_x = 0.0
        self._prev_time_x = None
        self._prev_error_y = 0.0
        self._integral_y = 0.0
        self._prev_time_y = None

        self._imu_subscriber = self.create_subscription(
            Imu,
            imu_topic,
            self._imu_callback,
            10
        )
        self._command_publisher = self.create_publisher(
            Float64MultiArray,
            command_topic,
            10
        )

        self.get_logger().info("Head Stabilizer Node started.")

    def _imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        try:
            (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list, 'sxyz')
        except Exception as e:
            self.get_logger().error(f"Failed to convert quaternion to Euler: {e}")
            return

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Process X axis (roll)
        if self._prev_time_x is None:
            dt_x = 0.0
        else:
            dt_x = current_time - self._prev_time_x

        error_x = -roll  # Desired angle is 0, error = 0 - actual
        self._integral_x += error_x * dt_x
        # Anti-windup for integral term
        self._integral_x = max(min(self._integral_x, self._integral_max_x), -self._integral_max_x)
        derivative_x = (error_x - self._prev_error_x) / dt_x if dt_x > 0 else 0.0

        output_x = self._Kp_x * error_x + self._Ki_x * self._integral_x + self._Kd_x * derivative_x

        # Process Y axis (pitch)
        if self._prev_time_y is None:
            dt_y = 0.0
        else:
            dt_y = current_time - self._prev_time_y

        error_y = -pitch  # Desired angle is 0, error = 0 - actual
        self._integral_y += error_y * dt_y
        # Anti-windup for integral term
        self._integral_y = max(min(self._integral_y, self._integral_max_y), -self._integral_max_y)
        derivative_y = (error_y - self._prev_error_y) / dt_y if dt_y > 0 else 0.0

        output_y = self._Kp_y * error_y + self._Ki_y * self._integral_y + self._Kd_y * derivative_y

        # Clamp outputs to joint limits
        clamped_x = max(-self._limit_x, min(output_x, self._limit_x))
        clamped_y = max(-self._limit_y, min(output_y, self._limit_y))

        # Publish command
        command_msg = Float64MultiArray()
        command_msg.data = [clamped_x, clamped_y]
        self._command_publisher.publish(command_msg)

        # Update previous values for next iteration
        self._prev_error_x = error_x
        self._prev_time_x = current_time
        self._prev_error_y = error_y
        self._prev_time_y = current_time

        # Optional debug logging
        # self.get_logger().debug(f"Cmd Angles: [{clamped_x:.3f}, {clamped_y:.3f}]")

def main(args=None):
    rclpy.init(args=args)
    try:
        head_stabilizer_node = HeadStabilizer()
        rclpy.spin(head_stabilizer_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'head_stabilizer_node' in locals() and rclpy.ok():
            head_stabilizer_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()