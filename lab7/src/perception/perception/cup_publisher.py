import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO


class CupDetector(Node):
    def __init__(self):
        super().__init__('cup_detector')

        self.bridge = CvBridge()
        self.camera_intrinsics = None

        # Load YOLO (pretrained)
        self.model = YOLO("yolov8n.pt")

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 1)

        self.cup_pub = self.create_publisher(
            PointStamped, '/cup_point', 1)

        self.get_logger().info("Cup detector initialized")

    def camera_info_callback(self, msg):
        self.camera_intrinsics = msg.k
        self.get_logger().info("Camera intrinsics received")

    def image_callback(self, msg):
        if self.camera_intrinsics is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, verbose=False)

        # YOLO results
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls.cpu().numpy()[0])
                cls_name = self.model.names[cls_id]

                # Only process cups
                if cls_name != "cup":
                    continue

                # Bounding box
                x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]

                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0

                # Approximate depth from bounding box height
                box_height = (y2 - y1)
                f_y = self.camera_intrinsics[4]

                # This constant MUST be tuned experimentally.
                REAL_CUP_HEIGHT = 0.12  # 12 cm cup

                depth = (REAL_CUP_HEIGHT * f_y) / box_height

                # Convert to camera coordinates
                f_x = self.camera_intrinsics[0]
                c_x = self.camera_intrinsics[2]
                c_y = self.camera_intrinsics[5]

                X = ((u - c_x) * depth) / f_x
                Y = ((v - c_y) * depth) / f_y
                Z = depth

                # Publish cup point
                p = PointStamped()
                p.header.stamp = msg.header.stamp
                p.header.frame_id = "camera_link"

                p.point.x = X
                p.point.y = Y
                p.point.z = Z

                self.cup_pub.publish(p)

                self.get_logger().info(
                    f"Detected cup at: X={X:.2f} Y={Y:.2f} Z={Z:.2f}"
                )

                return   # Only publish first (largest) cup detected


def main(args=None):
    rclpy.init(args=args)
    node = CupDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
