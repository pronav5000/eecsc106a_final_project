import numpy as np
from tf2_ros import TransformException
import tf2_geometry_msgs
from sensor_msgs.msg import JointState
import rclpy

def quaternion_to_y_axis_xy(qx, qy, qz, qw):
    """
    Given a quaternion (x, y, z, w) for the EE in base_link,
    return the EE's local +Y axis expressed in base_link, projected to XY.
    """
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-8:
        return np.array([0.0, 1.0])
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    r01 = 2.0 * (qx*qy - qz*qw)
    r11 = 1.0 - 2.0 * (qx*qx + qz*qz)

    yx = r01
    yy = r11
    v = np.array([yx, yy], dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-8:
        return np.array([0.0, 1.0])
    return v / n


def rotate_xy(vec, angle):
    """Rotate a 2D vector by angle (radians) around the origin."""
    c = np.cos(angle)
    s = np.sin(angle)
    x, y = vec
    return np.array([c*x - s*y, s*x + c*y], dtype=float)


def line_distance_to_point(p_e, d, p_c):
    """
    Distance from point p_c to the ray starting at p_e and going along d.
    p_e, d, p_c are 2D numpy arrays.
    d is assumed to be unit length.
    """
    w = p_c - p_e
    s = np.dot(d, w)
    if s <= 0:
        return np.linalg.norm(w) + 1e3

    perp = w - s * d
    return np.linalg.norm(perp)


def align_base(self):

    if self.joint_state is None:
        self.get_logger().error("No joint state available! Can't align base.")
        return

    if self.cup_pose is None:
        self.get_logger().error("No cup pose available! Can't align base.")
        return

    cup_pose_base = self.cup_pose
    cp = cup_pose_base.pose.position
    p_c_xy = np.array([cp.x, cp.y], dtype=float)

    try:
        tf_ee = self.tf_buffer.lookup_transform(
            'base_link',
            'wrist_3_link',
            rclpy.time.Time()
        )
    except TransformException as e:
        self.get_logger().error(
            f"TF error getting EE pose in base_link: {e}"
        )
        return

    ee_t = tf_ee.transform.translation
    ee_r = tf_ee.transform.rotation

    p_e0_xy = np.array([ee_t.x, ee_t.y], dtype=float)

    d0_xy = quaternion_to_y_axis_xy(
        ee_r.x, ee_r.y, ee_r.z, ee_r.w
    )

    base_idx = 5

    theta0 = float(self.joint_state.position[base_idx])
    theta_cup = np.arctan2(p_c_xy[1], p_c_xy[0])
    theta_ee_forward = np.arctan2(d0_xy[1], d0_xy[0])

    delta_guess = theta_cup - theta_ee_forward
    theta_guess = theta0 + delta_guess

    window = np.pi / 2.0
    num_samples = 181

    best_theta = theta_guess
    best_err = float('inf')

    for i in range(num_samples):
        alpha = -window + (2.0 * window) * (i / (num_samples - 1))
        theta_candidate = theta_guess + alpha
        delta = theta_candidate - theta0

        p_e_xy = rotate_xy(p_e0_xy, delta)
        d_xy = rotate_xy(d0_xy, delta)

        n_d = np.linalg.norm(d_xy)
        if n_d < 1e-8:
            continue
        d_xy = d_xy / n_d

        err = line_distance_to_point(p_e_xy, d_xy, p_c_xy)

        if err < best_err:
            best_err = err
            best_theta = theta_candidate

    self.get_logger().info(
        f"align_base (brute): θ0={theta0:.3f}, θ_guess={theta_guess:.3f}, "
        f"θ_best={best_theta:.3f}, min_err={best_err:.4f}"
    )

    target = JointState()
    target.header = self.joint_state.header
    target.name = list(self.joint_state.name)
    target.position = list(self.joint_state.position)

    target.position[base_idx] = best_theta

    traj = self.ik_planner.plan_to_joints(target)
    if traj is None:
        self.get_logger().error("Planning failed in align_base (brute).")
        return

    self._execute_joint_trajectory(traj.joint_trajectory)
