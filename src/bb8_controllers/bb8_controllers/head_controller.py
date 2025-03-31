#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import tf_transformations
import math

class HeadStabilizer(Node):
    """
    A ROS 2 node that subscribes to an IMU topic (/imu_base)
    and publishes joint commands to stabilize the robot's head.
    It commands the head joints to counteract the base's roll and pitch.
    """
    def __init__(self):
        super().__init__('head_stabilizer_node')

        # --- Parameters ---
        self.declare_parameter('imu_topic', '/imu_base')
        self.declare_parameter('command_topic', '/head_controller/commands')
        self.declare_parameter('joint_names', ['head_x_joint', 'head_y_joint'])

        self.declare_parameter('limit_x_lower', -math.pi / 9.0)
        self.declare_parameter('limit_x_upper', math.pi / 9.0)
        self.declare_parameter('limit_y_lower', -math.pi / 9.0)
        self.declare_parameter('limit_y_upper', math.pi / 9.0)

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self._joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self._limit_x_lower = self.get_parameter('limit_x_lower').get_parameter_value().double_value
        self._limit_x_upper = self.get_parameter('limit_x_upper').get_parameter_value().double_value
        self._limit_y_lower = self.get_parameter('limit_y_lower').get_parameter_value().double_value
        self._limit_y_upper = self.get_parameter('limit_y_upper').get_parameter_value().double_value

        if len(self._joint_names) != 2:
            self.get_logger().error("Expected 2 joint names in 'joint_names' parameter.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Subscribing to IMU on: {imu_topic}")
        self.get_logger().info(f"Publishing commands to: {command_topic}")
        self.get_logger().info(f"Controlling joints: {self._joint_names}")
        self.get_logger().info(f"Joint X limits: [{self._limit_x_lower:.3f}, {self._limit_x_upper:.3f}] rad")
        self.get_logger().info(f"Joint Y limits: [{self._limit_y_lower:.3f}, {self._limit_y_upper:.3f}] rad")


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
        """
        Processes incoming IMU message and publishes joint commands.
        """
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        try:
            (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list, 'sxyz')
        except Exception as e:
             self.get_logger().error(f"Failed to convert quaternion to Euler: {e}")
             return


        # --- Calculate Target Joint Angles ---
        target_x_angle = -roll
        target_y_angle = -pitch

        # --- Clamp angles to joint limits ---
        clamped_x_angle = max(self._limit_x_lower, min(target_x_angle, self._limit_x_upper))
        clamped_y_angle = max(self._limit_y_lower, min(target_y_angle, self._limit_y_upper))

        # --- Prepare and Publish Command ---
        command_msg = Float64MultiArray()

        command_msg.data = [clamped_x_angle, clamped_y_angle]

        self._command_publisher.publish(command_msg)

        # Log the calculated angles for debugging
        # self.get_logger().debug(f"Base RPY: [{roll:.3f}, {pitch:.3f}, {yaw:.3f}] -> Cmd Angles: [{clamped_x_angle:.3f}, {clamped_y_angle:.3f}]")


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
        # Cleanup
        if 'head_stabilizer_node' in locals() and rclpy.ok():
            head_stabilizer_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()