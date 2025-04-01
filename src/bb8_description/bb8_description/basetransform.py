import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class ImuToTf(Node):
    def __init__(self):
        super().__init__('imu_to_tf')
        self.subscription = self.create_subscription(
            Imu,
            '/imu_base',  # Topic where IMU data is published
            self.imu_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sphere_radius = 0.25

    def imu_callback(self, msg):
        # Extract orientation from IMU
        orientation = msg.orientation

        # Create transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.sphere_radius
        t.transform.rotation = orientation

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