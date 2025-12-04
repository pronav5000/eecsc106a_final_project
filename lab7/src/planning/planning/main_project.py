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


class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        # Subscribers
        self.cube_pub = self.create_subscription(
            PointStamped, '/cube_pose_base', self.cube_callback, 1)  # TODO: CHECK TOPIC
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 1)
        self.cup_sub = self.create_subscription(
            PointStamped, '/cup_pose_base', self.cup_callback, 1)

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
        self.job_queue = []  # Entries: JointState or 'toggle_grip' or ['throw_ball', x,y,z]

        # ACTIVE action/trajectory tracking for timed releases
        self._active_goal_handle = None
        self._traj_start_time = None                 # rclpy.time.Time
        self._pending_release_time = None            # seconds into trajectory (float)
        self._release_triggered = False

        # Timer to check pending release (runs even while action executing)
        self.create_timer(0.01, self._check_release_timer)

    # -----------------------
    # Callbacks
    # -----------------------
    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def cube_callback(self, cube_pose):
        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cube_pose = cube_pose

    def cup_callback(self, cup_pose):
        if self.cup_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cup_pose = cup_pose

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

        self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
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
    # Throw implementation (no threads)
    # -----------------------
    def _throw_ball(self, job):
        # job = ['throw_ball', x, y, z]
        cmd, x, y, z = job

        # compute IK for the throw target (uses your IK planner signature)
        target_js = self.ik_planner.compute_ik(self.joint_state, x, y, z)
        if target_js is None:
            self.get_logger().error("IK failed for throw pose")
            # continue queue
            self.execute_jobs()
            return

        # Build a short, fast two-point trajectory (current -> target)
        jt = JointTrajectory()
        jt.joint_names = list(self.joint_state.name)

        # point 0 = current positions (time 0)
        p0 = JointTrajectoryPoint()
        p0.positions = list(self.joint_state.position)
        p0.time_from_start = Duration(sec=0, nanosec=0)

        # point 1 = target positions at a short duration (fast throw)
        throw_duration = 0.40  # seconds; tune this for faster/slower throws
        p1 = JointTrajectoryPoint()
        p1.positions = list(target_js.position)
        p1.time_from_start = Duration(sec=0, nanosec=int(throw_duration * 1e9))

        # compute velocities for p1 (simple finite difference)
        dt = throw_duration if throw_duration > 1e-6 else 0.4
        velocities = []
        for a, b in zip(p0.positions, p1.positions):
            velocities.append((b - a) / dt)
        p1.velocities = velocities
        # optional accelerations (set to zero)
        p1.accelerations = [0.0] * len(velocities)

        jt.points = [p0, p1]

        # Schedule release time (fraction through the throw)
        total_time = throw_duration
        release_fraction = 0.7
        release_time = total_time * release_fraction

        self.get_logger().info(f"Throw: total_time={total_time:.3f}s, release at {release_time:.3f}s")

        # set pending release; the _check_release_timer will fire it once the trajectory starts
        self._pending_release_time = release_time
        self._release_triggered = False

        # send the trajectory (non-blocking). _on_goal_sent will set traj start time.
        self._execute_joint_trajectory(jt)

        # Do NOT call execute_jobs() here — we will continue when the action completes (_on_exec_done)
        # The release will be handled by the periodic timer while the action is executing.

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
            self.get_logger().info(f"Timed release triggered at elapsed={elapsed:.3f}s (scheduled {self._pending_release_time:.3f}s)")
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

        # optional: add a done callback to log result
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

        # clear active goal and start time
        self._active_goal_handle = None
        self._traj_start_time = None
        self._pending_release_time = None
        self._release_triggered = False

        # proceed to next job in queue
        self.execute_jobs()

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
