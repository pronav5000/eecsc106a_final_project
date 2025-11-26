import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped 
from scipy.spatial.transform import Rotation as R
import numpy as np

class TransformCubePose(Node):
    def __init__(self):
        super().__init__('transform_cube_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cube_pose_sub = self.create_subscription(
            PointStamped,
            '/cube_pose',
            self.cube_pose_callback,
            10
        )

        self.cube_pose_pub = self.create_publisher(
            PointStamped,
            '/cube_pose_base',
            100
        ) # Please ensure this is filled

        rclpy.spin_once(self, timeout_sec=2)
        self.cube_pose = None

    def cube_pose_callback(self, msg: PointStamped):
        if self.cube_pose is None:
            self.cube_pose = self.transform_cube_pose(msg)
        else:
            self.cube_pose_pub.publish(self.cube_pose)

    def transform_cube_pose(self, msg: PointStamped):
        """ 
        Transform point into base_link frame
        Args: 
            - msg: PointStamped - The message from /cube_pose, of the position of the cube in camera_depth_optical_frame
        Returns:
            Point: point in base_link_frame in form [x, y, z]
        """
        try: 
            tf_trans = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
        except:
            return
        quat = [tf_trans.transform.rotation.x,
        tf_trans.transform.rotation.y, tf_trans.transform.rotation.z, tf_trans.transform.rotation.w]
        rotation = R.from_quat(quat).as_matrix()
        translation = tf_trans.transform.translation
        point = msg.point

        trans = np.array([translation.x, translation.y, translation.z])
        point = np.array([point.x, point.y, point.z])

        transfomed_point = rotation @ point + trans

        return_msg = PointStamped()
        return_msg.header.stamp = self.get_clock().now().to_msg()
        return_msg.header.frame_id = 'base_link'
        return_msg.point.x = transfomed_point[0]
        return_msg.point.y = transfomed_point[1]
        return_msg.point.z = transfomed_point[2]


        return return_msg

def main(args=None):
    rclpy.init(args=args)
    node = TransformCubePose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
