import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class WheelOdom(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.declare_parameter('wheel_separation', 0.3641)
        self.declare_parameter('wheel_radius', 0.069)
        self.declare_parameter('angular_scale', 2.5)
        self.declare_parameter('linear_scale', 1.5)
        self.declare_parameter('publish_tf', True) # Default to True, but set to False via launch file when using EKF

        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.publish_tf_ = self.get_parameter('publish_tf').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._current_left_pos = 0.0
        self._current_right_pos = 0.0
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.last_time = self.get_clock().now()
        self.initialized = False

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        if self.publish_tf_:
            self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/50.0, self.update_odometry) # 50 Hz

        # --- Covariance parameters ---
        # Based on how much you trust your wheel odometry. Lower values = more trust.
        self.declare_parameter('cov_x', 0.001)
        self.declare_parameter('cov_y', 0.001)
        self.declare_parameter('cov_yaw', 0.01)
        self.declare_parameter('cov_vx', 0.002)
        self.declare_parameter('cov_vyaw', 0.008)

        self.cov_x = self.get_parameter('cov_x').value
        self.cov_y = self.get_parameter('cov_y').value
        self.cov_yaw = self.get_parameter('cov_yaw').value
        self.cov_vx = self.get_parameter('cov_vx').value
        self.cov_vyaw = self.get_parameter('cov_vyaw').value
        self.get_logger().info(f"WheelOdom initialized. Publish TF: {self.publish_tf_}")


    def joint_state_callback(self, msg):
        try:
            # Make sure these joint names match your URDF/robot
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            # self.get_logger().warn("left_wheel_joint or right_wheel_joint not in JointState. Skipping.", throttle_duration_sec=5)
            return

        self.current_left_pos = msg.position[left_idx]
        self.current_right_pos = msg.position[right_idx]

        if not self.initialized:
            self.prev_left_pos = self.current_left_pos
            self.prev_right_pos = self.current_right_pos
            self.initialized = True
            self.last_time = self.get_clock().now()

    def update_odometry(self):
        if not self.initialized:
            return

        current_time = self.get_clock().now()
        delta_time_s = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if delta_time_s <= 0: # Avoid division by zero
            # self.get_logger().warn("Delta time is non-positive, skipping odometry update.")
            return

        # Wheel rotations since last update
        delta_left_rad = self.current_left_pos - self.prev_left_pos
        delta_right_rad = self.current_right_pos - self.prev_right_pos

        # Distance each wheel traveled
        delta_left_dist = delta_left_rad * self.wheel_radius
        delta_right_dist = delta_right_rad * self.wheel_radius

        self.prev_left_pos = self.current_left_pos
        self.prev_right_pos = self.current_right_pos

        delta_s_robot_frame = (delta_left_dist + delta_right_dist) / 2.0
        delta_theta_robot_frame = (delta_right_dist - delta_left_dist) / self.wheel_separation

        # Apply scaling
        delta_s_scaled = delta_s_robot_frame / self.linear_scale
        delta_theta_scaled = delta_theta_robot_frame / self.angular_scale

        
        linear_velocity_x = delta_s_scaled / delta_time_s
        angular_velocity_z = delta_theta_scaled / delta_time_s

        # Update pose using the velocities (this is one way to do it)
        # This matches how EKF might integrate if it only had velocities
        self.x += linear_velocity_x * math.cos(self.theta) * delta_time_s
        self.y += linear_velocity_x * math.sin(self.theta) * delta_time_s
        self.theta += angular_velocity_z * delta_time_s
        
        # Normalize theta to be between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = linear_velocity_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity_z

        # --- Populate Covariances ---
        # Pose covariance (x, y, z, roll, pitch, yaw)
        odom.pose.covariance = np.zeros(36, dtype=np.float64)
        odom.pose.covariance[0] = self.cov_x  # X variance
        odom.pose.covariance[7] = self.cov_y  # Y variance
        odom.pose.covariance[35] = self.cov_yaw # Yaw variance
        # For 2D, other main diagonal terms for Z, roll, pitch can be small positive if fixed, or larger if "don't know"
        odom.pose.covariance[14] = 1e-9 # Z
        odom.pose.covariance[21] = 1e-9 # Roll
        odom.pose.covariance[28] = 1e-9 # Pitch


        # Twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
        odom.twist.covariance = np.zeros(36, dtype=np.float64)
        odom.twist.covariance[0] = self.cov_vx   # Vx variance
        odom.twist.covariance[35] = self.cov_vyaw # Vyaw variance
        # For 2D, other main diagonal terms for Vy, Vz, Vroll, Vpitch can be small positive if near zero, or larger if "don't know"
        odom.twist.covariance[7] = 1e-3 # Vy (diff drive doesn't measure this, but some small value)
        odom.twist.covariance[14] = 1e-9 # Vz
        odom.twist.covariance[21] = 1e-9 # Vroll
        odom.twist.covariance[28] = 1e-9 # Vpitch


        self.odom_pub.publish(odom)

        if self.publish_tf_:
            transform = TransformStamped()
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = q[0]
            transform.transform.rotation.y = q[1]
            transform.transform.rotation.z = q[2]
            transform.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(transform)

    @property
    def current_left_pos(self):
        return self._current_left_pos

    @current_left_pos.setter
    def current_left_pos(self, value):
        self._current_left_pos = value

    @property
    def current_right_pos(self):
        return self._current_right_pos

    @current_right_pos.setter
    def current_right_pos(self, value):
        self._current_right_pos = value

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()