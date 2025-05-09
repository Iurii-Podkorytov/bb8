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
        
        self.declare_parameter('max_z_deviation_output', 0.5) # For filtering points in final base_link Z
        self.max_z_dev = self.get_parameter('max_z_deviation_output').value

        self.imu_sub = Subscriber(self, Imu, '/imu_head') # IMU providing orientation of the Lidar
        self.scan_sub = Subscriber(self, LaserScan, '/scan') # Original Lidar scan
        
        self.ts = ApproximateTimeSynchronizer([self.imu_sub, self.scan_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.get_logger().info(f"ScanFilter node started. Sphere radius: {self.sphere_radius}, Max Z dev: {self.max_z_dev}")
        
    def sync_callback(self, imu_msg, scan_msg):
        self.get_logger().debug("Sync callback triggered.")
        
        q = imu_msg.orientation
        # euler_from_quaternion gives roll (about X), pitch (about Y), yaw (about Z)
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        R_x_correct = np.array([
            [1, 0, 0],
            [0, np.cos(-roll), -np.sin(-roll)],
            [0, np.sin(-roll), np.cos(-roll)]
        ])
        R_y_correct = np.array([
            [np.cos(-pitch), 0, np.sin(-pitch)],
            [0, 1, 0],
            [-np.sin(-pitch), 0, np.cos(-pitch)]
        ])
        # Combined rotation matrix from Lidar frame to base_link frame
        R_base_from_lidar = R_y_correct @ R_x_correct
        
        new_ranges = []
        
        offset = np.array([0.0, 0.0, -self.sphere_radius])

        for i in range(len(scan_msg.ranges)):
            r = scan_msg.ranges[i]
            
            # Handle invalid readings (inf, nan, or outside physical Lidar range)
            if not (scan_msg.range_min <= r <= scan_msg.range_max):
                new_ranges.append(float('inf'))
                continue
            
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Point in Lidar's Cartesian frame
            x_lidar = r * math.cos(angle)
            y_lidar = r * math.sin(angle)
            z_lidar = 0.0
            point = np.array([x_lidar, y_lidar, z_lidar])
            
            # Point coordinates relative to the pivot point P, still in Lidar's frame
            translated = point - offset

            # Rotate this vector into the base_gootprint frame
            rotated = R_base_from_lidar @ translated
            # This matches your 'rotated' variable.

            # Get point coordinates in base_footprint frame.
            final_point = rotated + offset
            
            x_base = final_point[0]
            y_base = final_point[1]
            z_base = final_point[2]
            
            # Optional: Filter points too far from base_link's XY plane
            if abs(z_base) > self.max_z_dev :
                new_ranges.append(float('inf'))
                continue

            # Calculate the 2D range in the XY plane of base_footprint
            new_range_2d = math.sqrt(x_base**2 + y_base**2)
            
            # Check if the new 2D range is valid
            if scan_msg.range_min <= new_range_2d <= scan_msg.range_max:
                new_ranges.append(new_range_2d)
            else:
                new_ranges.append(float('inf'))
        
        transformed_scan = LaserScan()
        transformed_scan.header = scan_msg.header # Keep original timestamp for TF purposes
        transformed_scan.header.frame_id = 'base_footprint'
        
        transformed_scan.angle_min = scan_msg.angle_min
        transformed_scan.angle_max = scan_msg.angle_max
        transformed_scan.angle_increment = scan_msg.angle_increment
        transformed_scan.time_increment = scan_msg.time_increment
        transformed_scan.scan_time = scan_msg.scan_time
        transformed_scan.range_min = scan_msg.range_min
        transformed_scan.range_max = scan_msg.range_max
        transformed_scan.ranges = new_ranges
        
        self.scan_pub.publish(transformed_scan)
        # self.get_logger().info(f"Published transformed scan. First new range: {new_ranges[0] if new_ranges else 'N/A'}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ScanFilter node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()