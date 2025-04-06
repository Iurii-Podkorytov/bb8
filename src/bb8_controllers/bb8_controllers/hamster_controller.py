import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class HamsterController(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheels')
        
        self.declare_parameter('wheel_separation', 0.3641)
        self.declare_parameter('wheel_radius', 0.069)
        
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        self.command_publisher = self.create_publisher(Float64MultiArray, '/wheels_controller/commands', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        left_vel = (2 * msg.linear.x - self.wheel_separation * msg.angular.z) / (2 * self.wheel_radius)
        right_vel = (2 * msg.linear.x + self.wheel_separation * msg.angular.z) / (2 * self.wheel_radius)
        
        command_msg = Float64MultiArray()
        command_msg.data = [left_vel, right_vel]
        self.command_publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HamsterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()