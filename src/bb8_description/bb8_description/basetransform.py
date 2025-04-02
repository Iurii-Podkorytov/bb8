import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class ImuToTf(Node):
    def __init__(self):
        super().__init__('base_transform')
        self.subscription = self.create_subscription(
            Imu,
            '/imu_base',
            self.imu_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sphere_radius = 0.25

    def imu_callback(self, msg):
        # Extract orientation from IMU
        orientation = msg.orientation

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w],
            axes='sxyz'  # X: roll, Y: pitch, Z: yaw
        )

        # Create a new quaternion with only the pitch (Y-axis) component
        new_quat = tf_transformations.quaternion_from_euler(
            0.0,  # Roll (X-axis)
            euler[1],  # Pitch (Y-axis)
            0.0,  # Yaw (Z-axis)
            axes='sxyz'
        )

        # Create transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.sphere_radius
        t.transform.rotation.x = new_quat[0]
        t.transform.rotation.y = new_quat[1]
        t.transform.rotation.z = new_quat[2]
        t.transform.rotation.w = new_quat[3]

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()