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
        self.declare_parameter('model_path', 'yolov8l.pt')
        self.declare_parameter('ball_class', 'sports ball')
        self.declare_parameter('image_topic', '/camera1/image_raw')
        self.declare_parameter('camera_info_topic', '/camera1/camera_info')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.ball_class = self.get_parameter('ball_class').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.ball_pub = self.create_publisher(PointStamped, "/ball_pos", 100)

        self.model = YOLO(model_path)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 1)

        self.get_logger().info("Ball detector initialized")
        self.bridge = CvBridge()

    def camera_info_callback(self, msg: CameraInfo):
        if len(msg.k) >= 9:
            k = msg.k
            self.fx = float(k[0])
            self.fy = float(k[4])
            self.cx = float(k[2])
            self.cy = float(k[5])
            self.get_logger().info("Camera intrinsics received")
        else:
            self.get_logger().warn("CameraInfo received but K not filled.")


    def image_callback(self, msg):
        if not hasattr(self, 'fx'):
            return
        self.get_logger().info("Start of image callback")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = cv_image.shape[:2]
        cv_image = cv_image.reshape(h, w, 2)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUY2)
        cv_image = np.ascontiguousarray(cv_image)
        #scale_factor = 4
        #cv_image = cv2.resize(cv_image, (cv_image.shape[1]*scale_factor, cv_image.shape[0]*scale_factor))
        
        results = self.model(cv_image, verbose=False)

        self.get_logger().info("Got cv image")
        
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls.cpu().numpy()[0])
                cls_name = self.model.names[cls_id]
                if cls_name != self.ball_class:
                    continue
                
                x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0
                box_height = max(1.0, (y2 - y1))

                print(u, v)

                diameter = 0.07
                depth = (diameter * self.fy) / box_height
                X = ((u - self.cx) * depth) / self.fx
                Y = ((v - self.cy) * depth) / self.fy
                Z = depth

                p = PointStamped()
                p.header = msg.header
                p.header.frame_id = "camera_link"
                p.point.x = float(X)
                p.point.y = float(Y)
                p.point.z = float(Z)

                self.ball_pub.publish(p)
                self.get_logger().info(f"Ball in camera frame: X={X:.2f} Y={Y:.2f} Z={Z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
