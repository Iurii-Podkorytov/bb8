import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        
        self.declare_parameter('sphere_radius', 0.26)
        self.sphere_radius = self.get_parameter('sphere_radius').value
        
        self.imu_sub = Subscriber(self, Imu, '/imu_base')
        self.scan_sub = Subscriber(self, LaserScan, '/scan')
        
        self.ts = ApproximateTimeSynchronizer([self.imu_sub, self.scan_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.scan_pub = self.create_publisher(LaserScan, '/transformed_scan', 10)
        
        
    def sync_callback(self, imu_msg, scan_msg):
        # Rotate the scan based on IMU orientation
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()