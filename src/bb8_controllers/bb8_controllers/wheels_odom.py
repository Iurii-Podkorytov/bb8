import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import TransformStamped
import math

class WheelOdom(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        self.declare_parameter('wheel_separation', 0.3641)
        self.declare_parameter('wheel_radius', 0.069)
        self.declare_parameter('angular_scale', 2.5)
        self.declare_parameter('linear_scale', 1.5)
        self.declare_parameter('publish_tf', True)
        
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.last_time = self.get_clock().now()
        self.initialized = False
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0/50.0, self.update_odometry)

    def joint_state_callback(self, msg):
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            return
        
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
        
        delta_left = (self.current_left_pos - self.prev_left_pos) * self.wheel_radius
        delta_right = (self.current_right_pos - self.prev_right_pos) * self.wheel_radius
        
        self.prev_left_pos = self.current_left_pos
        self.prev_right_pos = self.current_right_pos
        
        delta_theta = (delta_right - delta_left) / self.wheel_separation / self.angular_scale
        delta_s = (delta_left + delta_right) / 2.0 / self.linear_scale
        
        theta_avg = self.theta + delta_theta / 2.0
        
        self.x += delta_s * math.cos(theta_avg)
        self.y += delta_s * math.sin(theta_avg)
        self.theta += delta_theta
        
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
        
        linear_velocity = delta_s / delta_time
        angular_velocity = delta_theta / delta_time
        
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        
        self.odom_pub.publish(odom)
        
        if self.publish_tf:
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()