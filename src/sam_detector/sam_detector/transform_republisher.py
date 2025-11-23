import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

class TransformRepublisher(Node):
    def __init__(self):
        super().__init__("transform_republisher")

        # TF utilities
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Periodically republish the transform
        self.timer = self.create_timer(0.1, self.publish_from_base)

    def publish_from_base(self):
        try:
            # Look up the transform that SAMDetector publishes
            trans = self.tf_buffer.lookup_transform(
                target_frame="base_link",
                source_frame="gripper_target_object",
                time=Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            # Broadcast the same transform (now parented to base_link)

            tf = TransformStamped()
            tf.header.frame_id = 'base_link'
            tf.child_frame_id = 'gripper_target_object_moveit'
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.transform = trans.transform
            self.tf_broadcaster.sendTransform(tf)
        except Exception as e:
            # Transform not available yet – ignore and try again
            self.get_logger().error(f"Transform lookup failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TransformRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()