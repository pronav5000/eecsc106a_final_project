#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import numpy as np


class CupPosePublisher(Node):
    def __init__(self):
        super().__init__("cup_pose_publisher")

        # not sure if i need these lines

        self.declare_parameter("marker_id", 'ar_marker_20')
        self.declare_parameter("marker_id_2", 'ar_marker_10')
        #self.declare_parameter("target_frame", "base_link")

        self.marker_id = self.get_parameter("marker_id").get_parameter_value().string_value
        self.marker_id_2 = self.get_parameter("marker_id_2").get_parameter_value().string_value
        #self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PointStamped, "/cup_point", 10)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f"Publishing cup pose as TF({self.marker_id_2} → {self.marker_id})"
        )

        self.samples = []

    def loop(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.marker_id_2,
                self.marker_id,
                rclpy.time.Time()
            )

            p = PointStamped()
            p.header.frame_id = self.marker_id_2
            p.header.stamp = self.get_clock().now().to_msg()

            pt = np.array([
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z
            ], dtype=float)

            self.samples.append(pt)

            if len(self.samples) > 10:
                self.samples.pop(0)
            
            if len(self.samples) < 10:
                return
            
            avg = np.mean(self.samples, axis=0)
            p.point.x = avg[0]
            p.point.y = avg[1]
            p.point.z = avg[2]


            # pose = PoseStamped()
            # pose.header.stamp = self.get_clock().now().to_msg()
            # pose.header.frame_id = self.marker_id_2

            # pose.pose.position.x = tf_msg.transform.translation.x
            # pose.pose.position.y = tf_msg.transform.translation.y
            # pose.pose.position.z = tf_msg.transform.translation.z
            # pose.pose.orientation = tf_msg.transform.rotation

            self.pub.publish(p)
            #self.get_logger().info(f"Ball in camera frame: X={p.point.x:.2f} Y={p.point.y:.2f} Z={p.point.z:.2f}")


        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = CupPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
