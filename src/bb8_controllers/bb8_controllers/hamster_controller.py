import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
import math

class HamsterController(Node):
    def __init__(self):
        super().__init__('hamster_controller')
        
        # Robot parameters
        self.wheel_separation = 0.3641
        self.wheel_radius = 0.069
        self.sphere_radius = 0.26

        self.angular_scale = 2.5
        self.linear_scale = 1.5

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.last_time = self.get_clock().now()
        self.initialized = False

        self.declare_parameter('publish_tf', True)    # Publish TF transform

        # Wheel velocity command publishers
        self._command_publisher = self.create_publisher(Float64MultiArray, '/wheels_controller/commands', 10)

        # Odometry subscribers/publishers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Velocity command subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Timer for odometry updates (50 Hz)
        self.timer = self.create_timer(1.0/50.0, self.update_odometry)

    def cmd_vel_callback(self, msg):
        # Convert to wheel velocities
        left_vel = (2 * msg.linear.x - self.wheel_separation * msg.angular.z) / (2 * self.wheel_radius)
        right_vel = (2 * msg.linear.x + self.wheel_separation * msg.angular.z) / (2 * self.wheel_radius)

        # Publish to wheel controllers
        command_msg = Float64MultiArray()
        command_msg.data = [left_vel, right_vel]
        self._command_publisher.publish(command_msg)

    def joint_state_callback(self, msg):
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        # Update current positions
        self.current_left_pos = msg.position[left_idx]
        self.current_right_pos = msg.position[right_idx]

        if not self.initialized:
            self.prev_left_pos = self.current_left_pos
            self.prev_right_pos = self.current_right_pos
            self.initialized = True

    def update_odometry(self):
        if not self.initialized:
            return

        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate wheel displacements
        delta_left = (self.current_left_pos - self.prev_left_pos) * self.wheel_radius
        delta_right = (self.current_right_pos - self.prev_right_pos) * self.wheel_radius

        # Update previous positions for next iteration
        self.prev_left_pos = self.current_left_pos
        self.prev_right_pos = self.current_right_pos

        # Compute odometry
        delta_theta = (delta_right - delta_left) / self.wheel_separation / self.angular_scale
        delta_s = (delta_left + delta_right) / 2.0 / self.linear_scale

        # Calculate average angle during the interval
        theta_avg = self.theta + delta_theta / 2.0

        # Update position and orientation
        self.x += delta_s * math.cos(theta_avg)
        self.y += delta_s * math.sin(theta_avg)
        self.theta += delta_theta

        # Create quaternion from yaw
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # Publish odometry message
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

        # Calculate linear and angular velocities
        linear_velocity = delta_s / delta_time
        angular_velocity = delta_theta / delta_time

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        # Publish transform
        if self.get_parameter('publish_tf').value:
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

    # Properties to track wheel positions
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
    node = HamsterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()