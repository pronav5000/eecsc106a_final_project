#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class CupPosePublisher(Node):
    def __init__(self, base_frame, ar_marker):
        super().__init__("cup_pose_publisher")

        # not sure if i need these lines

        # self.declare_parameter("marker_id", 7)
        # self.declare_parameter("target_frame", "base_link")

        # self.marker_id = self.get_parameter("marker_id").get_parameter_value().integer_value
        # self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value

        self.base_frame = base_frame
        self.ar_marker = ar_marker

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, "/cup_pose", 10)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f"Publishing cup pose as TF({self.base_frame} → {self.ar_marker})"
        )

    def loop(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ar_marker,
                rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.base_frame

            pose.pose.position.x = tf_msg.transform.translation.x
            pose.pose.position.y = tf_msg.transform.translation.y
            pose.pose.position.z = tf_msg.transform.translation.z
            pose.pose.orientation = tf_msg.transform.rotation

            self.pub.publish(pose)

        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: ros2 run my_pkg cup_pose_publisher base_frame ar_ar_marker")
        rclpy.shutdown()
        return

    base_frame = sys.argv[1]
    ar_marker = sys.argv[2]

    node = CupPosePublisher(base_frame, ar_marker)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
