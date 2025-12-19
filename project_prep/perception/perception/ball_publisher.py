#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import numpy as np


class BallPointPublisher(Node):
    """
        Node that publishes PointStamped of the ball expressed in AR marker frame,
        to topic '/ball_point'.
    """

    def __init__(self):
        super().__init__("ball_pose_publisher")

        # declare marker_id params
        self.declare_parameter('ball_marker_id', 'ar_marker_21')
        self.declare_parameter('base_marker_id', 'ar_marker_10')
        self.ball_marker_id = self.get_parameter('ball_marker_id').get_parameter_value().string_value
        self.base_marker_id = self.get_parameter('base_marker_id').get_parameter_value().string_value

        # set up tf listeners and publisher
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointStamped, '/ball_point', 10)
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f"Publishing ball point as TF({self.base_marker_id} → {self.ball_marker_id})"
        )

        self.samples = []

    def loop(self):
        """
            Estimate translations and publish PointStamped.
        """
        try:
            ball_point = PointStamped()
            ball_point.header.frame_id = self.base_marker_id
            ball_point.header.stamp = self.get_clock().now().to_msg()

            self.estimate_translation()
            ball_point.point.x, ball_point.point.y, ball_point.point.z = np.mean(self.samples, axis=0)

            self.pub.publish(ball_point)

        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")
    
    def estimate_translation(self):
        """
            Estimates the translation from ball AR marker to base AR marker, by
            averaging 10 samples of the transform. Returns None.
        """
        # obtain transform
        tf = self.tf_buffer.lookup_transform(
                self.base_marker_id,
                self.ball_marker_id,
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

    node = BallPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
