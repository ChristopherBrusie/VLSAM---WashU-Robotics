import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        static_transform = TransformStamped()

        # Set the header
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        #static_transform.header.frame_id = 'odom'

        # Set the child frame
        static_transform.child_frame_id = 'camera_link'

        # Set the translation and rotation
        static_transform.transform.translation.x = 0.115 #will be -0.1152 from camera_link
        static_transform.transform.translation.y = 0.0  
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Publish the transform
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Publishing static transform from base_link to camera_link')


def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

