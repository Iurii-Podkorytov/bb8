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
        self.linear_scale = 7
        self.angular_scale = 1.5

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.last_time = self.get_clock().now()
        self.initialized = False

        # Wheel velocity command publishers
        self._command_publisher = self.create_publisher(Float64MultiArray, '/wheels_controller/commands', 10)

        # Odometry subscribers/publishers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheels', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Velocity command subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_smoothed',
            self.cmd_vel_callback,
            10)

        # Timer for odometry updates (50 Hz)
        self.timer = self.create_timer(1.0/50.0, self.update_odometry)

    def cmd_vel_callback(self, msg):
        # Scale velocities
        scaled_linear = msg.linear.x
        scaled_angular = msg.angular.z

        # Convert to wheel velocities
        left_vel = (2 * scaled_linear - self.wheel_separation * scaled_angular) / (2 * self.wheel_radius)
        right_vel = (2 * scaled_linear + self.wheel_separation * scaled_angular) / (2 * self.wheel_radius)

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
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        # Calculate wheel displacements
        delta_left = (self.current_left_pos - self.prev_left_pos) * self.wheel_radius * self.linear_scale
        delta_right = (self.current_right_pos - self.prev_right_pos) * self.wheel_radius * self.linear_scale

        # Compute motion
        linear = (delta_left + delta_right) / 2.0
        angular = (delta_right - delta_left) / (self.wheel_separation * self.angular_scale)

        # Integrate pose
        delta_x = linear * math.cos(self.theta) * dt
        delta_y = linear * math.sin(self.theta) * dt
        delta_theta = angular * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Update previous positions
        self.prev_left_pos = self.current_left_pos
        self.prev_right_pos = self.current_right_pos
        self.last_time = current_time

        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # Turn off odom transform, it will come from ekf
        # self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = t.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        try:
            odom.twist.twist.linear.x = linear / dt
            odom.twist.twist.angular.z = angular / dt
            self.odom_pub.publish(odom)
        except ZeroDivisionError as e:
            self.get_logger().error('dt = 0')

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