import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from tf_transformations import euler_from_quaternion
import math

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        
        self.declare_parameter('sphere_radius', 0.26)
        self.sphere_radius = self.get_parameter('sphere_radius').value
        
        self.imu_sub = Subscriber(self, Imu, '/imu_head')
        self.scan_sub = Subscriber(self, LaserScan, '/scan')
        
        self.ts = ApproximateTimeSynchronizer([self.imu_sub, self.scan_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.scan_pub = self.create_publisher(LaserScan, '/transformed_scan', 10)
        
    def sync_callback(self, imu_msg, scan_msg):
        q = imu_msg.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Rotation matrices for -roll around x and -pitch around y
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), np.sin(roll)],
            [0, -np.sin(roll), np.cos(roll)]
        ])
        R_y = np.array([
            [np.cos(pitch), 0, -np.sin(pitch)],
            [0, 1, 0],
            [np.sin(pitch), 0, np.cos(pitch)]
        ])
        R = R_y @ R_x  # Combine rotations: first X, then Y
        
        new_ranges = []
        for i in range(len(scan_msg.ranges)):
            r = scan_msg.ranges[i]
            if r < scan_msg.range_min or r > scan_msg.range_max:
                new_ranges.append(float('inf'))
                continue
            
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0
            
            # Translate to sphere center (0,0,-sphere_radius)
            translated = np.array([x, y, z + self.sphere_radius])
            
            # Apply rotation
            rotated = R @ translated
            
            # Translate back
            final_point = rotated - np.array([0, 0, self.sphere_radius])
            
            # Compute new range and angle
            new_range = np.linalg.norm(final_point)
            if new_range < scan_msg.range_min or new_range > scan_msg.range_max:
                new_ranges.append(float('inf'))
            else:
                new_ranges.append(new_range)
        
        transformed_scan = LaserScan()
        transformed_scan.header = scan_msg.header
        transformed_scan.header.frame_id = 'base_link'
        transformed_scan.angle_min = scan_msg.angle_min
        transformed_scan.angle_max = scan_msg.angle_max
        transformed_scan.angle_increment = scan_msg.angle_increment
        transformed_scan.time_increment = scan_msg.time_increment
        transformed_scan.scan_time = scan_msg.scan_time
        transformed_scan.range_min = scan_msg.range_min
        transformed_scan.range_max = scan_msg.range_max
        transformed_scan.ranges = new_ranges
        
        self.scan_pub.publish(transformed_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()