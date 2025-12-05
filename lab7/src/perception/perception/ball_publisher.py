import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        self.bridge = CvBridge()
        self.camera_intrinsics = None

        # Load YOLO model
        self.model = YOLO("yolov8n.pt")

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 1)

        # Publisher
        self.ball_pub = self.create_publisher(
            PointStamped, '/ball_point', 1)

        self.get_logger().info("Ball detector initialized")

    def camera_info_callback(self, msg):
        self.camera_intrinsics = msg.k
        self.get_logger().info_once("Camera intrinsics received")

    def image_callback(self, msg):
        if self.camera_intrinsics is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, verbose=False)

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls.cpu().numpy()[0])
                cls_name = self.model.names[cls_id]

                # YOLO class names for balls depend on model
                if cls_name is not "ball":
                    continue

                # Bounding box center
                x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0

                # Bounding box height (pixels)
                box_height = (y2 - y1)

                # Focal length
                f_y = self.camera_intrinsics[4]

                # *** YOU MUST TUNE THIS ***
                REAL_BALL_DIAMETER = 0.07  # 7 cm ball

                # Approx depth (similar triangles)
                depth = (REAL_BALL_DIAMETER * f_y) / box_height

                # Convert (u,v,depth) → 3D point in camera frame
                f_x = self.camera_intrinsics[0]
                c_x = self.camera_intrinsics[2]
                c_y = self.camera_intrinsics[5]

                X = ((u - c_x) * depth) / f_x
                Y = ((v - c_y) * depth) / f_y
                Z = depth

                # Publish ball point
                p = PointStamped()
                p.header.stamp = msg.header.stamp
                p.header.frame_id = "camera_link"

                p.point.x = X
                p.point.y = Y
                p.point.z = Z

                self.ball_pub.publish(p)

                self.get_logger().info(
                    f"Ball detected at camera frame: "
                    f"X={X:.2f} Y={Y:.2f} Z={Z:.2f}"
                )

                return  # only use strongest detection


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
