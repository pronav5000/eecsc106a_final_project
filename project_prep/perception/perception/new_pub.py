#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


def blob_detect_position(cv_image, thr_min, thr_max, blur=5, search_window=None):
    """Detect the largest blob in the image and return its centroid (u, v) and pixel height."""
    h, w = cv_image.shape[:2]

    # Restrict to a search window if provided
    if search_window:
        x_min = int(search_window[0] * w)
        y_min = int(search_window[1] * h)
        x_max = int(search_window[2] * w)
        y_max = int(search_window[3] * h)
        mask_window = np.zeros((h, w), dtype=np.uint8)
        mask_window[y_min:y_max, x_min:x_max] = 255
    else:
        mask_window = None

    # Ensure blur kernel is odd
    k = blur if blur % 2 == 1 else blur + 1
    img_blur = cv2.GaussianBlur(cv_image, (k, k), 0)
    hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(thr_min, dtype=np.uint8),
        np.array(thr_max, dtype=np.uint8)
    )

    if mask_window is not None:
        mask = cv2.bitwise_and(mask, mask_window)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None

    # Take largest contour
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    if M['m00'] == 0:
        return None

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    box_height = cv2.boundingRect(c)[3]  # pixel height
    return cx, cy, box_height


class Ball3DDetector(Node):
    def __init__(self):
        super().__init__('ball_3d_detector')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('ball_diameter', 0.07)  # meters
        self.declare_parameter('hsv_min', [55, 40, 0])
        self.declare_parameter('hsv_max', [150, 255, 255])
        self.declare_parameter('blur', 5)

        self.ball_diameter = self.get_parameter('ball_diameter').value
        self.hsv_min = self.get_parameter('hsv_min').value
        self.hsv_max = self.get_parameter('hsv_max').value
        self.blur = self.get_parameter('blur').value

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 1)
        self.pub = self.create_publisher(PointStamped, '/ball_pos', 10)

        self.get_logger().info("Ball 3D detector initialized")

    def camera_info_callback(self, msg: CameraInfo):
        if len(msg.k) >= 9:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info("Camera intrinsics received")

    def image_callback(self, msg: Image):
        if any(v is None for v in (self.fx, self.fy, self.cx, self.cy)):
            # wait for camera info
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        result = blob_detect_position(cv_image, self.hsv_min, self.hsv_max, self.blur)
        if result is None:
            return  # no ball detected

        u, v, box_height = result

        # Simple pinhole camera model to compute depth
        depth = (self.ball_diameter * self.fy) / max(box_height, 1.0)
        X = ((u - self.cx) * depth) / self.fx
        Y = ((v - self.cy) * depth) / self.fy
        Z = depth

        # Publish 3D position
        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp
        point_msg.header.frame_id = 'camera_link'
        point_msg.point.x = float(X)
        point_msg.point.y = float(Y)
        point_msg.point.z = float(Z)

        self.pub.publish(point_msg)
        self.get_logger().info(f"Ball pos: X={X:.2f} Y={Y:.2f} Z={Z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = Ball3DDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
