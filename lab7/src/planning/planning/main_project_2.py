#!/usr/bin/env python3

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

from builtin_interfaces.msg import Duration
from planning.ik import IKPlanner


def bezier_curve(p0, p1, p2, p3, t):
    """Cubic Bézier interpolation between 4 control points (np.array)."""
    return ((1.0 - t) ** 3) * p0 \
        + 3.0 * ((1.0 - t) ** 2) * t * p1 \
        + 3.0 * (1.0 - t) * (t ** 2) * p2 \
        + (t ** 3) * p3


class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        # Subscribers
        self.ball_pub = self.create_subscription(
            PointStamped, '/ball_point', self.cube_callback, 1
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 1
        )
        self.cup_sub = self.create_subscription(
            PointStamped, '/cup_point', self.cup_callback, 1
        )

        # Action client for joint trajectories
        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        # Gripper service
        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        # State
        self.cube_pose = None
        self.cup_pose = None
        self.current_plan = None
        self.joint_state = None

        self.ik_planner = IKPlanner()

        # Single unified queue (keeps your structure)
        # Entries: JointState or 'toggle_grip' or ['throw_ball', x, y, z]
        self.job_queue = []

        # ACTIVE action/trajectory tracking for timed releases
        self._active_goal_handle = None
        self._traj_start_time = None           # rclpy.time.Time
        self._pending_release_time = None      # seconds into trajectory (float)
        self._release_triggered = False

        # Throw configuration (you can tune these)
        # Fixed release position in base frame
        self.release_pos = np.array([0.6, 0.0, 0.4], dtype=float)
        # How far past release to follow through along throw direction
        self.follow_through_dist = 0.08  # meters

        # Timer to check pending release during trajectory execution
        self.create_timer(0.01, self._check_release_timer)

    # -----------------------
    # Callbacks
    # -----------------------
    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def cube_callback(self, cube_pose: PointStamped):
        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cube_pose = cube_pose

    def cup_callback(self, cup_pose: PointStamped):
        if self.cup_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cup_pose = cup_pose

        # Precompute throw poses (release + follow‑through)
        self.pose_release, self.pose_end = self._compute_throw_poses(self.cup_pose)

        # -----------------------------------------------------------
        # Build job queue exactly like your original code
        # -----------------------------------------------------------

        # 1) Move to Pre-Grasp Position (gripper above the cube)
        x = self.cube_pose.point.x
        y = self.cube_pose.point.y - 0.035
        z = self.cube_pose.point.z + 0.185
        state_1 = self.ik_planner.compute_ik(self.joint_state, x, y, z)
        self.job_queue.append(state_1)

        # 2) Move to Grasp Position (lower the gripper to the cube)
        x = self.cube_pose.point.x
        y = self.cube_pose.point.y - 0.027
        z = self.cube_pose.point.z + 0.16
        state_2 = self.ik_planner.compute_ik(self.joint_state, x, y, z)
        self.job_queue.append(state_2)

        # 3) Close the gripper.
        self.job_queue.append('toggle_grip')

        # 4) Move back to Pre-Grasp Position
        x = self.cube_pose.point.x
        y = self.cube_pose.point.y - 0.035
        z = self.cube_pose.point.z + 0.185
        state_3 = self.ik_planner.compute_ik(self.joint_state, x, y, z)
        self.job_queue.append(state_3)

        # 5) Throw preparation: add throw job (single entry)
        # (x, y, z here are unused in _throw_ball but we keep them to match your structure)
        x = self.cup_pose.point.x + 0.4
        y = self.cup_pose.point.y
        z = self.cup_pose.point.z + 0.18
        self.job_queue.append(['throw_ball', x, y, z])

        # NOTE: do NOT append a separate 'toggle_grip' here — the throw job
        # will schedule the mid-trajectory release itself.

        # Start executing the queue
        self.execute_jobs()

    # -----------------------
    # Job executor (keeps your structure)
    # -----------------------
    def execute_jobs(self):
        if not self.job_queue:
            self.get_logger().info("All jobs completed.")
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"Executing job queue, {len(self.job_queue)} jobs remaining."
        )
        next_job = self.job_queue.pop(0)

        if isinstance(next_job, JointState):
            traj = self.ik_planner.plan_to_joints(next_job)
            if traj is None:
                self.get_logger().error("Failed to plan to position")
                # continue to next job
                self.execute_jobs()
                return

            self.get_logger().info("Planned to position")
            self._execute_joint_trajectory(traj.joint_trajectory)

        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper (queued, blocking)")
            # keep blocking behavior for queued gripper toggle
            self._toggle_gripper()

        elif isinstance(next_job, list) and next_job[0] == 'throw_ball':
            self.get_logger().info("Throwing Ball job received")
            self._throw_ball(next_job)

        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    # -----------------------
    # Throw implementation (Bezier + no threads)
    # -----------------------
    def _throw_ball(self, job):
        # job = ['throw_ball', x, y, z]  (x,y,z unused here)
        _cmd, _x, _y, _z = job

        if self.cup_pose is None:
            self.get_logger().error("No cup pose available for throw.")
            self.execute_jobs()
            return

        # 1) Compute release + follow-through poses based on current cup position
        pose_rel, pose_end = self._compute_throw_poses(self.cup_pose)

        # 2) Bezier positions between release and follow‑through
        bezier_positions = self.compute_throw_bezier_points(
            pose_rel, pose_end, offset=0.05, num_points=8
        )

        # 3) IK for each Bezier position → joint waypoints
        joint_waypoints = []
        for p in bezier_positions:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            js = self.ik_planner.compute_ik(self.joint_state, x, y, z)
            if js is None:
                self.get_logger().warn("IK failed for a Bezier waypoint, skipping it")
                continue
            joint_waypoints.append(js)

        if len(joint_waypoints) < 2:
            self.get_logger().error(
                "Not enough IK waypoints for throw; aborting throw job"
            )
            self.execute_jobs()
            return

        # 4) Build one JointTrajectory from these joint waypoints
        jt = JointTrajectory()
        jt.joint_names = list(joint_waypoints[0].name)

        total_throw_time = 0.40  # seconds, tune this for throw speed
        n = len(joint_waypoints)
        dt = total_throw_time / (n - 1)

        for i, js in enumerate(joint_waypoints):
            pt = JointTrajectoryPoint()
            pt.positions = list(js.position)
            pt.time_from_start = Duration(
                sec=0,
                nanosec=int(i * dt * 1e9)
            )
            jt.points.append(pt)

        # simple velocity estimate for the last point
        if n >= 2:
            last = jt.points[-1]
            prev = jt.points[-2]
            vel = []
            for a, b in zip(prev.positions, last.positions):
                vel.append((b - a) / dt)
            last.velocities = vel

        # 5) Choose release time ~ halfway along the Bezier path
        release_index = int(0.5 * (n - 1))
        release_time = release_index * dt
        self._pending_release_time = release_time
        self._release_triggered = False

        self.get_logger().info(
            f"Throw trajectory with {n} waypoints, total_time={total_throw_time:.2f}s, "
            f"release at t={release_time:.2f}s (idx={release_index})"
        )

        # 6) Send the single joint trajectory to the controller
        self._execute_joint_trajectory(jt)
        # Do NOT call execute_jobs() here — _on_exec_done will advance the queue

    # -----------------------
    # Release timer checker
    # -----------------------
    def _check_release_timer(self):
        # This runs periodically regardless of whether a trajectory is executing.
        if self._pending_release_time is None:
            return
        if self._release_triggered:
            return
        if self._traj_start_time is None:
            return

        now = self.get_clock().now()
        elapsed = (now - self._traj_start_time).nanoseconds * 1e-9
        if elapsed >= self._pending_release_time:
            self.get_logger().info(
                f"Timed release triggered at elapsed={elapsed:.3f}s "
                f"(scheduled {self._pending_release_time:.3f}s)"
            )
            # Non-blocking release (do not call execute_jobs from here)
            self._toggle_gripper_async()
            self._release_triggered = True
            # Clear pending after firing
            self._pending_release_time = None

    # -----------------------
    # Gripper helpers
    # -----------------------
    def _toggle_gripper(self):
        # Blocking gripper toggle used for queued toggle_grip entries
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds (blocking) because this was queued
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done() and future.result() is not None:
            self.get_logger().info('Gripper toggled (blocking).')
        else:
            self.get_logger().error('Gripper toggle service failed or timed out.')

        # Continue queue after blocking gripper call
        self.execute_jobs()  # Proceed to next job

    def _toggle_gripper_async(self):
        # Non-blocking gripper toggle used for mid-trajectory release
        if not self.gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Gripper service not available (async)')
            return

        req = Trigger.Request()
        fut = self.gripper_cli.call_async(req)

        def _grip_done_cb(future):
            if future.done() and future.result() is not None:
                self.get_logger().info('Gripper toggled (async).')
            else:
                self.get_logger().warn('Gripper async call failed or timed out.')

        fut.add_done_callback(_grip_done_cb)

    # -----------------------
    # Trajectory sending & callbacks (store goal handle + start time)
    # -----------------------
    def _execute_joint_trajectory(self, joint_traj: JointTrajectory):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().error('Trajectory was rejected by server')
            # continue queue to avoid deadlock
            self.execute_jobs()
            return

        # store active handle and set start time
        self._active_goal_handle = goal_handle
        self._traj_start_time = self.get_clock().now()
        self.get_logger().info('Trajectory accepted and executing.')

        # attach done callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            _result = future.result().result
            self.get_logger().info('Execution complete.')
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')

        # clear active goal and start time / release flags
        self._active_goal_handle = None
        self._traj_start_time = None
        self._pending_release_time = None
        self._release_triggered = False

        # proceed to next job in queue
        self.execute_jobs()

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

    # -----------------------
    # Optional helper if you need to wait for a trajectory to finish
    # (not used in this design because we rely on _on_exec_done)
    # -----------------------
    def _wait_for_trajectory_done(self):
        if self._active_goal_handle is None:
            return
        future = self._active_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)


# -----------------------
# Main
# -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
