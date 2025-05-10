import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RedCircleDetector(Node):
    def __init__(self):
        super().__init__('red_circle_detector')
        
        # Camera parameters
        self.camera_frame = 'camera'
        self.base_frame = 'base_link'
        self.map_frame = 'map'
        self.horizontal_fov = 1.0472  # 60 degrees in radians
        self.image_width = 640         # Camera resolution
        
        # Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            10)
        
        self.publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)
        
        self.bridge = CvBridge()
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Latest LiDAR scan
        self.latest_scan = None

    def scan_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        if self.latest_scan is None:
            self.get_logger().warn("No LiDAR scan received yet.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        detected, area, (x, y) = self.detect_red_circle(cv_image)
        if not detected:
            return

        try:
            # Get transform from base_link to camera
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Transform error: {ex}')
            return

        # Calculate angle from image center
        cx = self.image_width / 2
        pixel_offset = x - cx
        angle_in_camera = (pixel_offset / (self.image_width / 2)) * (self.horizontal_fov / 2)

        # Optional: Apply camera-to-lidar offset (if camera is not aligned with LiDAR)
        lidar_angle = angle_in_camera  # Modify if needed

        # Get LiDAR range at the computed angle
        distance = self.get_lidar_distance_at_angle(lidar_angle)
        if distance is None:
            return  # No valid range

        # Compute direction vector in camera frame
        dir_camera = np.array([
            np.sin(lidar_angle),  # X-axis
            0,                    # Y-axis
            np.cos(lidar_angle)   # Z-axis
        ])

        # Normalize
        dir_camera /= np.linalg.norm(dir_camera)

        # Rotate to base_link frame
        quat = transform.transform.rotation
        rot_matrix = self.quaternion_to_rotation_matrix([
            quat.x, quat.y, quat.z, quat.w])
        
        dir_base_link = np.dot(rot_matrix, dir_camera)

        # Get camera position in base_link
        cam_pos = transform.transform.translation

        # Calculate goal in base_link frame
        goal_base_link = np.array([
            cam_pos.x + dir_base_link[0] * distance,
            cam_pos.y + dir_base_link[1] * distance,
            cam_pos.z + dir_base_link[2] * distance
        ])

        try:
            # Transform to map frame
            map_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Map transform error: {ex}')
            return

        quat_map = map_transform.transform.rotation
        rot_map = self.quaternion_to_rotation_matrix([
            quat_map.x, quat_map.y, quat_map.z, quat_map.w])
        
        trans_map = map_transform.transform.translation

        goal_map = np.dot(rot_map, goal_base_link) + [
            trans_map.x, trans_map.y, trans_map.z]
        
        # Publish goal
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = goal_map[0]
        pose_msg.pose.position.y = goal_map[1]
        pose_msg.pose.position.z = 0.0  # On floor
        pose_msg.pose.orientation.w = 1.0  # Simple orientation
        
        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Goal detected at: ({goal_map[0]:.2f}, {goal_map[1]:.2f})')

    def get_lidar_distance_at_angle(self, angle):
        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment

        # Clamp angle to [angle_min, angle_max]
        if angle < angle_min or angle > angle_max:
            self.get_logger().warn(f"Angle {angle:.3f} out of LiDAR range [{angle_min:.3f}, {angle_max:.3f}]")
            return None

        # Find nearest index
        idx = int((angle - angle_min) / angle_increment)
        if idx < 0 or idx >= len(scan.ranges):
            return None

        # Get the range
        range_val = scan.ranges[idx]
        if range_val < scan.range_min or range_val > scan.range_max:
            self.get_logger().warn(f"Invalid range: {range_val:.3f}")
            return None

        return range_val

    def detect_red_circle(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return (False, 0, (0,0))

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)

        if radius < 5:
            return (False, 0, (0,0))

        return (True, area, (int(x), int(y)))

    def quaternion_to_rotation_matrix(self, q):
        x, y, z, w = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

def main(args=None):
    rclpy.init()
    detector = RedCircleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()