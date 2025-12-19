import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import numpy as np


class CupPointPublisher(Node):
    def __init__(self):
        super().__init__("cup_point_publisher")

        self.declare_parameter('cup_marker_id', 'ar_marker_20')
        self.declare_parameter('base_marker_id', 'ar_marker_10')

        self.cup_marker_id = self.get_parameter('cup_marker_id').get_parameter_value().string_value
        self.base_marker_id = self.get_parameter('base_marker_id').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointStamped, '/cup_point', 10)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f"Publishing cup point as TF({self.base_marker_id} → {self.cup_marker_id})"
        )

        self.samples = []

    def loop(self):
        """
            Estimate translations and publish PointStamped.
        """
        try:
            cup_point = PointStamped()
            cup_point.header.frame_id = self.base_marker_id
            cup_point.header.stamp = self.get_clock().now().to_msg()

            self.estimate_translation()
            cup_point.point.x, cup_point.point.y, cup_point.point.z = np.mean(self.samples, axis=0)

            self.pub.publish(cup_point)

        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")

    def estimate_translation(self):
        """
            Estimates the translation from cup AR marker to base AR marker, by
            averaging 10 samples of the transform. Returns None.
        """
        # obtain transform
        tf = self.tf_buffer.lookup_transform(
                self.base_marker_id,
                self.cup_marker_id,
                rclpy.time.Time())
        
        # obtain translation encoded within transform
        trans = tf.transform.translation
        trans = np.array([trans.x, trans.y, trans.z], dtype=float)
        self.samples.append(trans)

        # limit sample size to 10
        if len(self.samples) > 10:
            self.samples.pop(0)
        elif len(self.samples) < 10:
            return

def main(args=None):
    rclpy.init(args=args)

    node = CupPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
