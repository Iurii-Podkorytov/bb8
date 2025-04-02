import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityProfileNode(Node):
    def __init__(self):
        super().__init__('velocity_profile_node')
        
        # Control parameters
        self.declare_parameter('linear_accel', 0.5)  # m/s²
        self.declare_parameter('linear_decel', 0.5)  # m/s²
        self.declare_parameter('angular_accel', 1.0) # rad/s²
        self.declare_parameter('angular_decel', 1.0) # rad/s²

        # Current state
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_time = self.get_clock().now()

        # Subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.smooth_cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_smoothed',
            10)
        
        # Control loop timer (50 Hz)
        self.timer = self.create_timer(1.0/50.0, self.control_loop)

    def cmd_vel_callback(self, msg):
        # Update target velocities from incoming command
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Linear velocity control
        delta_linear = self.target_linear - self.current_linear
        if delta_linear > 0:
            accel = self.get_parameter('linear_accel').value
            self.current_linear += min(delta_linear, accel * dt)
        elif delta_linear < 0:
            decel = self.get_parameter('linear_decel').value
            self.current_linear += max(delta_linear, -decel * dt)

        # Angular velocity control
        delta_angular = self.target_angular - self.current_angular
        if delta_angular > 0:
            accel = self.get_parameter('angular_accel').value
            self.current_angular += min(delta_angular, accel * dt)
        elif delta_angular < 0:
            decel = self.get_parameter('angular_decel').value
            self.current_angular += max(delta_angular, -decel * dt)

        # Publish smoothed command
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.smooth_cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityProfileNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()