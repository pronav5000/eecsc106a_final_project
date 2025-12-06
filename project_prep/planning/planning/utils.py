# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np
import time


class UR7e_Helper(Node):
    def __init__(self):
        super().__init__('calc_helper') ####what do i do with this

        # Subscribers
        self.ball_pub = self.create_subscription(
            PointStamped, '/ball_pos', self.cube_callback, 1
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 1
        )
        self.cup_sub = self.create_subscription(
            PointStamped, '/cup_pose', self.cup_callback, 1
        )
     # State
        self.cube_pose = None
        self.cup_pose = None
        self.current_plan = None
        self.joint_state = None

    # -----------------------
    # Trajectory calculator helper functions
    # -----------------------

    def _compute_throw_poses(self, cup_pose_base: PointStamped):
        # 1) Read cup and release positions
        p_rel = self.release_pos  # np.array([x_rel, y_rel, z_rel])
        p_cup = np.array([
            cup_pose_base.point.x,
            cup_pose_base.point.y,
            cup_pose_base.point.z
        ], dtype=float)

        d = p_cup - p_rel          # vector from release point to cup
        d_norm = np.linalg.norm(d)
        if d_norm < 1e-6:
            self.get_logger().warn(
                "Cup too close to release point; using default direction."
            )
            d = np.array([1.0, 0.0, 0.0], dtype=float)
            d_norm = 1.0

        d_hat = d / d_norm         # unit direction toward cup

        # 2) Choose follow‑through endpoint past the release point
        p_end = p_rel + self.follow_through_dist * d_hat

        # 3) Compute yaw, pitch from direction vector d
        dx, dy, dz = d

        # yaw: rotation around z so x-axis faces cup in the x–y plane
        yaw = np.arctan2(dy, dx)

        # horizontal distance
        r_xy = np.hypot(dx, dy)

        # pitch: tilt up/down so x-axis points toward cup in x–z
        # negative sign because of zyx convention
        pitch = np.arctan2(-dz, r_xy)

        roll = 0.0  # no twist around the throwing axis (adjust if needed)

        # 4) Build quaternion from yaw, pitch, roll
        rot = R.from_euler('zyx', [yaw, pitch, roll])
        qx, qy, qz, qw = rot.as_quat()  # SciPy gives (x, y, z, w)

        # 5) Build release Pose (pose_rel)
        pose_rel = Pose()
        pose_rel.position.x = float(p_rel[0])
        pose_rel.position.y = float(p_rel[1])
        pose_rel.position.z = float(p_rel[2])
        pose_rel.orientation.x = float(qx)
        pose_rel.orientation.y = float(qy)
        pose_rel.orientation.z = float(qz)
        pose_rel.orientation.w = float(qw)

        # 6) Build follow‑through Pose (pose_end)
        pose_end = Pose()
        pose_end.position.x = float(p_end[0])
        pose_end.position.y = float(p_end[1])
        pose_end.position.z = float(p_end[2])
        pose_end.orientation.x = float(qx)
        pose_end.orientation.y = float(qy)
        pose_end.orientation.z = float(qz)
        pose_end.orientation.w = float(qw)

        return pose_rel, pose_end

    def compute_throw_bezier_points(self, pose_rel: Pose, pose_end: Pose,
                                    offset: float = 0.05, num_points: int = 10):
        # 1) Turn poses into 3D vectors
        p0 = np.array([
            pose_rel.position.x,
            pose_rel.position.y,
            pose_rel.position.z
        ], dtype=float)

        p3 = np.array([
            pose_end.position.x,
            pose_end.position.y,
            pose_end.position.z
        ], dtype=float)

        # 2) Direction of throw (from release toward follow-through)
        d = p3 - p0
        norm = np.linalg.norm(d)
        if norm < 1e-6:
            # if they’re too close, just choose some default direction
            d_hat = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            d_hat = d / norm

        # 3) Control points along the same direction
        #    - p1 is a bit "after" p0 in the throw direction
        #    - p2 is a bit "before" p3 in the throw direction
        p1 = p0 + offset * d_hat
        p2 = p3 - offset * d_hat

        # 4) Sample Bezier from t in [0, 1]
        t_vals = np.linspace(0.0, 1.0, num_points)
        positions = [bezier_curve(p0, p1, p2, p3, t) for t in t_vals]

        return positions  # list of np.array([x, y, z])

