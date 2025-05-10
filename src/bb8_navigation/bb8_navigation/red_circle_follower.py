import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
from tf2_ros import TransformListener, Buffer, TransformException
from rclpy.duration import Duration
import tf2_geometry_msgs

class RedCircleDetector(Node):
    def __init__(self):
        super().__init__('red_circle_detector')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('horizontal_fov_deg', 60.0)
        self.declare_parameter('image_width_px', 640)
        self.declare_parameter('goal_standoff_distance', 0.5) 

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.horizontal_fov = math.radians(self.get_parameter('horizontal_fov_deg').get_parameter_value().double_value)
        self.image_width = self.get_parameter('image_width_px').get_parameter_value().integer_value
        self.goal_standoff_distance = self.get_parameter('goal_standoff_distance').get_parameter_value().double_value

        self.image_sub = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_scan = None

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_lidar_distance_at_angle(self, target_angle_rad: float, scan: LaserScan) -> float | None:
        if scan is None:
            self.get_logger().warn("No scan data available for get_lidar_distance_at_angle.")
            return None
        
        min_angle = scan.angle_min
        max_angle = scan.angle_max
        increment = scan.angle_increment

        if not (min_angle <= target_angle_rad <= max_angle):
            self.get_logger().warn(
                f"Target angle {target_angle_rad:.3f} rad ({math.degrees(target_angle_rad):.1f} deg) "
                f"is outside LiDAR's range [{min_angle:.3f}, {max_angle:.3f}] rad."
            )
            return None

        index = int(round((target_angle_rad - min_angle) / increment))

        if 0 <= index < len(scan.ranges):
            distance = scan.ranges[index]
            if math.isinf(distance) or math.isnan(distance) or \
               distance < scan.range_min or distance > scan.range_max:
                self.get_logger().warn(
                    f"Invalid LiDAR reading at angle {target_angle_rad:.3f} rad (index {index}): {distance}. "
                    f"Valid range: [{scan.range_min:.2f}, {scan.range_max:.2f}]"
                )
                return None
            return distance
        else:
            self.get_logger().warn(
                f"Calculated index {index} for angle {target_angle_rad:.3f} rad is out of bounds "
                f"for LiDAR ranges (0 to {len(scan.ranges)-1})."
            )
            return None

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        q.w = cr * cp * cy + sr * sp * sy
        return q

    def image_callback(self, msg: Image):
        current_scan = self.latest_scan
        if current_scan is None:
            self.get_logger().warn("No LiDAR scan received yet. Skipping image processing.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        detected, _, (x_circle_center, _) = self.detect_red_circle(cv_image)

        if detected:
            pixel_offset_from_center = x_circle_center - (self.image_width / 2.0)
            angle_in_camera_fov_rad = (pixel_offset_from_center / (self.image_width / 2.0)) * (self.horizontal_fov / 2.0)
            target_angle_for_lidar_rad = -angle_in_camera_fov_rad
            
            distance = self.get_lidar_distance_at_angle(target_angle_for_lidar_rad, current_scan)

            if distance is not None and distance > 0:
                point_in_lidar_frame = PointStamped()
                point_in_lidar_frame.header.stamp = current_scan.header.stamp
                point_in_lidar_frame.header.frame_id = current_scan.header.frame_id
                
                point_in_lidar_frame.point.x = distance * math.cos(target_angle_for_lidar_rad)
                point_in_lidar_frame.point.y = distance * math.sin(target_angle_for_lidar_rad)
                point_in_lidar_frame.point.z = 0.0 

                try:
                    transform_lidar_to_map = self.tf_buffer.lookup_transform(
                        self.map_frame,
                        point_in_lidar_frame.header.frame_id,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.2)
                    )
                    
                    circle_pos_map = tf2_geometry_msgs.do_transform_point(
                        point_in_lidar_frame, transform_lidar_to_map
                    )
                    
                    goal_pose = PoseStamped()
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.header.frame_id = self.map_frame
                    
                    final_goal_position = Point()

                    try:
                        transform_robot_to_map = self.tf_buffer.lookup_transform(
                            self.map_frame,
                            self.robot_base_frame,
                            rclpy.time.Time(),
                            timeout=Duration(seconds=0.1)
                        )
                        robot_x_map = transform_robot_to_map.transform.translation.x
                        robot_y_map = transform_robot_to_map.transform.translation.y

                        vec_x = circle_pos_map.point.x - robot_x_map
                        vec_y = circle_pos_map.point.y - robot_y_map
                        dist_robot_to_circle = math.sqrt(vec_x**2 + vec_y**2)

                        if dist_robot_to_circle <= self.goal_standoff_distance or dist_robot_to_circle < 0.01:
                            final_goal_position = circle_pos_map.point
                        else:
                            ux = vec_x / dist_robot_to_circle
                            uy = vec_y / dist_robot_to_circle
                            dist_to_actual_goal_point = dist_robot_to_circle - self.goal_standoff_distance
                            
                            final_goal_position.x = robot_x_map + ux * dist_to_actual_goal_point
                            final_goal_position.y = robot_y_map + uy * dist_to_actual_goal_point
                            final_goal_position.z = circle_pos_map.point.z 
                    except TransformException as ex:
                        self.get_logger().warn(
                            f"Could not get robot pose for standoff calculation: {ex}. Setting goal at circle location.")
                        final_goal_position = circle_pos_map.point

                    goal_pose.pose.position = final_goal_position
                    goal_pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, 0.0)
                    
                    self.publisher.publish(goal_pose)
                    self.get_logger().info(
                        f"Published goal_pose: Pos({goal_pose.pose.position.x:.2f}, "
                        f"{goal_pose.pose.position.y:.2f}, {goal_pose.pose.position.z:.2f}) in '{self.map_frame}'."
                    )
                    
                except TransformException as ex:
                    self.get_logger().warn(
                        f"Could not transform point from '{point_in_lidar_frame.header.frame_id}' to '{self.map_frame}': {ex}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Error during pose creation: {e}", exc_info=True)

    def detect_red_circle(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70]); upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70]); upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2 
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2) 
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours: return (False, 0, (0, 0))
            
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < 50: return (False, 0, (0,0))

        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        
        if radius < 5: return (False, 0, (0, 0))
            
        return (True, area, (int(x), int(y)))
    
def main(args=None):
    rclpy.init(args=args)
    detector = RedCircleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()