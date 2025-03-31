import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class P3DBroadcaster(Node):
    def __init__(self):
        super().__init__('p3d_tf_broadcaster')
        
        # Create a TransformBroadcaster to broadcast transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/sphere_pose',
            self._callback,
            10 
        )
        self.subscription 

        self.get_logger().info('P3D Tf Broadcaster initialized.')

    def _callback(self, msg):
        """
        Callback function to handle incoming messages.
        Broadcasts the transform between base_footprint and sphere.
        """
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the header with the same timestamp and frame ID
        t.header = msg.header
        t.child_frame_id = 'sphere_link'

        # Set the translation (position)
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Set the rotation (orientation)
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = P3DBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()