# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped
from moveit_msgs.msg import RobotTrajectory, JointConstraint, Constraints
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from ur_msgs.srv import SetSpeedSliderFraction
import time
from scipy.optimize import brentq

#divya added
from new_align_base import align_base


from planning.ik import IKPlanner

class UR7e_Throw(Node):
    def __init__(self):
        super().__init__('ur7e_throw')

        # publishers
        self.ball_pub = self.create_subscription(PointStamped,
                                                 '/ball_point', self.ball_callback, 1) 
        self.joint_pub = self.create_publisher(JointTrajectory,
                                               '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        
        # subscribers
        self.cup_sub = self.create_subscription(PointStamped,
                                                '/cup_point', self.cup_callback, 1) 
        self.joint_state_sub = self.create_subscription(JointState,
                                                        '/joint_states', self.joint_state_callback, 1)

        # clients
        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.speed_slider_cli = self.create_client(
            SetSpeedSliderFraction, '/io_and_status_controller/set_speed_slider')
        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        # points (type: PointStamped)
        self.ball_point = None
        self.cup_point = None

        # fixed velocity (measured)
        self.v = 1.4 # m/s
        
        # joint states
        self.joint_state = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        self.ik_planner = IKPlanner() # inverse kinematics solver

        self.tf_buffer = Buffer() # tf buffer
        self.tf_listener = TransformListener(self.tf_buffer, self) # tf listener

        self.job_queue = [] # job queue
   
    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg
    
    def cup_callback(self, cup_point: PointStamped):
        """
            Transforms cup_point into frame 'base_link'.
            Stores result (type: PointStamped) in self.cup_point.
        """
        if cup_point is None:
            self.get_logger().info("No cup position, cannot proceed")
            return
        if self.cup_point is not None: return

        # ------------------------------------------------------------
        # Transforms to frame 'base_link'
        # ------------------------------------------------------------
        self.cup_point = self.transform_to_base_link(cup_point)
        if self.cup_point is None: return

        self.get_logger().info(
            f"Transformed cup position in base frame: X={self.cup_point.point.x} Y={self.cup_point.point.y} Z={self.cup_point.point.z}")

    def ball_callback(self, ball_point: PointStamped):
        """
            Transforms ball_point into frame 'base_link', stores result
            in self.ball_point, assembles job queue, and executes jobs.
        """
        if ball_point is None:
            self.get_logger().info("No ball position, cannot proceed")
            return
        if self.ball_point is not None: return
        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        # ------------------------------------------------------------
        # Transforms to frame 'base_link'
        # ------------------------------------------------------------
        self.ball_point = self.transform_to_base_link(ball_point)
        if self.ball_point is None: return

        b = self.ball_point.point
        self.get_logger().info(
            f"Transformed ball position in base frame: X={b.x} Y={b.y} Z={b.z}")

        # -----------------------------------------------------------------
        # checks if we have a self.cup_point
        # -----------------------------------------------------------------
        if self.cup_point is None:
            self.ball_point = None
            self.get_logger().info("No cup position, cannot proceed")
            return
        
        # -----------------------------------------------------------------
        # Creates job queue
        # -----------------------------------------------------------------
        self.create_job_queue()
        self.execute_jobs()

    def create_job_queue(self):
        """
        Appends jobs to queue for subsequent execution.
        """
        assert self.ball_point and self.cup_point
        self.get_logger().info("Starting joint solution calculation!")

        # Offsets for pre-grasp position
        b = self.ball_point.point
        dx, dy, dz = 0.018, -0.03, 0.25

        # ------------------------------------------------------------
        # 1) MOVE TO PRE-GRASP POSITION (ABOVE THE BALL)
        # ------------------------------------------------------------
        sol = self.ik_planner.compute_ik(self.joint_state, b.x+dx, b.y+dy, b.z+dz)
        self.job_queue.append(sol)

        # ------------------------------------------------------------
        # 2) MOVE TO GRASP POSITION (LOWER GRIPPER)
        # ------------------------------------------------------------
        sol = self.ik_planner.compute_ik(self.joint_state, b.x+dx, b.y+dy, b.z+dz-0.045)
        self.job_queue.append(sol)

        # ------------------------------------------------------------
        # 3) CLOSE GRIPPER
        # ------------------------------------------------------------
        self.job_queue.append('toggle_grip')

        # ------------------------------------------------------------
        # 4) RETURN TO TUCK POSITION
        # ------------------------------------------------------------
        self.job_queue.append('restore_state')

        # ------------------------------------------------------------
        # 5) WIND UP
        # ------------------------------------------------------------
        self.job_queue.append('wind_up')

        # ------------------------------------------------------------
        # 6/7) ALIGN BASE AND THROW BALL
        # ------------------------------------------------------------
        self.job_queue.append('throw_ball')

        self.job_queue.append('reset_speed')

    def wind_up(self):
        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't throw.")
            return

        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)

        # corresponds to a backrotation by ~30 degrees relative to vertical
        target.position = [-1.368044675593712,
                           0.23849994341005498,
                           -1.7610446415343226 + np.pi/2,
                           1.5493460893630981,
                           -3.1248191038714808,
                            self.joint_state.position[5]]
        traj = self.ik_planner.plan_to_joints(target)
        
        assert traj
        self._execute_joint_trajectory(traj.joint_trajectory)

    def align_base(self):
        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't align base.")
            return
        
        if self.cup_point is None:
            self.get_logger().error("No cup position available! Can't align base.")
            return

        # obtain arm length from base joint to shoulder joint
        try:
            g = self.tf_buffer.lookup_transform('shoulder_joint', 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")
            return
        
        c, t = self.cup_point.point, g.transform.translation
        a = np.linalg.norm([t.x, t.y, t.z]) # arm length
        
        # calculate dtheta required to align base to cup
        d = np.sqrt(np.linalg.norm([c.x, c.y])**2 - a**2)
        num = d*c.x + a*c.y
        denom = a*c.x + d*c.y
        dtheta = np.arctan2(num, denom)

        # setup target joint state
        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)

        i = target.name.index('shoulder_pan_joint')
        target.position[i] += dtheta
        
        # set up traj
        traj = self.ik_planner.plan_to_joints(target)
        assert traj
        
        self.get_logger().info("Planned base alignment, executing...")
        return traj.joint_trajectory

    def throw_ball(self, start_state: JointState):
        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't throw.")
            return
        
        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)

        target.position = [
            -3.16737618806883753,
            0.17614967027773076,
            -1.7613245449461878 + np.pi/2,
            1.549370527267456,
            -3.1248038450824183,
            self.joint_state.position[5]
        ]
      

        traj = self.ik_planner.plan_to_joints(target, start_state)
        assert traj
        return traj.joint_trajectory
    
    def align_and_throw(self, release_time: float):
        """
        Concatenates align and throw trajectories together (because
        MoveIt was accelerating and decelerating the motion path for
        some reason)
        """
        # create align to base trajectory
        align = self.align_base()
        assert align

        # intermediate joint state
        inter = JointState()
        inter.name = list(align.joint_names)
        inter.position = list(align[-1].positions)

        throw = self.throw_ball(inter)
        assert throw

        traj = self.concatenate_traj(align, throw)

        self._execute_joint_trajectory(traj)
        self.timer = self.create_timer(release_time, self.execute_gripper_toggle)
        
    def _traj_time_to_sec(self, d):
        return float(d.sec) + 1e-9 * float(d.nanosec)

    def concatenate_traj(self, t1: JointTrajectory, t2: JointTrajectory) -> JointTrajectory:
        if t1.joint_names != t2.joint_names:
            raise ValueError("Joint names mismatch; cannot concatenate")

        out = JointTrajectory()
        out.joint_names = list(t1.joint_names)
        out.points = []

        # copy jt1 points
        for p in t1.points:
            p2 = JointTrajectoryPoint()
            p2.positions = list(p.positions)
            p2.velocities = list(p.velocities) if p.velocities else []
            p2.accelerations = list(p.accelerations) if p.accelerations else []
            p2.effort = list(p.effort) if p.effort else []
            p2.time_from_start = p.time_from_start
            out.points.append(p2)

        if not out.points:
            return t2

        offset = self._traj_time_to_sec(out.points[-1].time_from_start)

        # skip first point of jt2 to avoid duplicate time/point
        start_idx = 1 if len(t2.points) > 0 else 0

        for p in t2.points[start_idx:]:
            t = self._traj_time_to_sec(p.time_from_start) + offset

            p2 = JointTrajectoryPoint()
            p2.positions = list(p.positions)
            p2.velocities = list(p.velocities) if p.velocities else []
            p2.accelerations = list(p.accelerations) if p.accelerations else []
            p2.effort = list(p.effort) if p.effort else []
            p2.time_from_start.sec = int(t)
            p2.time_from_start.nanosec = int((t - int(t)) * 1e9)
            out.points.append(p2)

        return out

    def restore_state(self):
        """
        Return robot to tucked position.
        """

        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't throw.")
            return
        
        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)
        target.position = [-1.8433, -1.4332, -1.3970, 1.5829, -3.1359, 4.7200]

        traj = self.ik_planner.plan_to_joints(target)
        assert traj
        self._execute_joint_trajectory(traj.joint_trajectory)

    def execute_gripper_toggle(self):
        # This function is called after 1 second
        self.get_logger().info("Toggling Gripper")
        self._toggle_gripper()
        self.timer.cancel()  # Cancel the timer after it triggers

        # Proceed with the next job in the queue
        self.execute_jobs()  # Cont

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
                return
            self.get_logger().info("Planned to position")

        match next_job:
            case 'toggle_grip':
                self.get_logger().info("Toggling gripper")
                self._toggle_gripper()
            case 'rotate_gripper':
                self.get_logger().info("Rotating gripper")
                self._rotate_gripper()
            case 'restore_state':
                self.get_logger().info("Restoring to tuck")
                self.restore_state()
            case 'throw_ball':
                self.get_logger().info("Aligning base and throwing ball")
                rt = self.release_time()
                self.align_and_throw(rt)
            case 'reset_speed':
                self.get_logger().info("Resetting speed")
                self.reset_speed()
            case 'wind_up':
                self.get_logger().info("Winding")
                self.wind_up()
            case _:
                self.get_logger().error("Unknown job type")
                self.execute_jobs()  # Proceed to next job

    def _toggle_gripper(self):
        if not self.gripper_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            rclpy.shutdown()
            return

        req = Trigger.Request()
        future = self.gripper_cli.call_async(req)
        # wait for 2 seconds
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        self.get_logger().info('Gripper toggled.')
        self.execute_jobs()  # Proceed to next job

    def _rotate_gripper(self, angle_rad=np.pi/2):
        """
        Rotate the wrist (last joint) by angle_rad (default +90 degrees)
        based on the current joint state, then plan and execute.
        """
        if self.joint_state is None:
            self.get_logger().error("No joint state available, cannot rotate gripper")
            self.execute_jobs()
            return

        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)

        i = target.name.index('wrist_3_joint')
        target.position[i] += angle_rad

        # Plan to this new joint configuration
        traj = self.ik_planner.plan_to_joints(target)
        if traj is None:
            self.get_logger().error("Failed to plan gripper rotation")
            self.execute_jobs()
            return

        self.get_logger().info("Planned gripper rotation, executing...")
        self._execute_joint_trajectory(traj.joint_trajectory)
    
    def scale_joint_trajectory_time(self, jt, scale: float):
        eps = 1e-6  # 1 microsecond to ensure monotonicity

        last_t = -1.0

        for p in jt.points:
            # convert to float seconds
            t = p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec

            # scale time
            t *= scale

            # enforce strictly increasing time
            if t <= last_t:
                t = last_t + eps
            last_t = t

            # write back
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)

            # scale derivatives
            if p.velocities:
                p.velocities = [v / scale for v in p.velocities]
            if p.accelerations:
                p.accelerations = [a / (scale * scale) for a in p.accelerations]
            
    def _execute_joint_trajectory(self, joint_traj):
        self.get_logger().info('Waiting for controller action server...')
        self.exec_ac.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        self.get_logger().info('Sending trajectory to controller...')
        send_future = self.exec_ac.send_goal_async(goal)
        print(send_future)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('bonk')
            rclpy.shutdown()
            return

        self.get_logger().info('Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, future):
        try:
            result = future.result().result
            self.get_logger().info('Execution complete.')
            self.execute_jobs()  # Proceed to next job
        except Exception as e:
            self.get_logger().error(f'Execution failed: {e}')

    def transform_to_base_link(self, point: PointStamped):
        """
            Transforms cup_point into frame 'base_link'.
            Returns result as a PointStamped.
        """
        try:
            g = self.tf_buffer.lookup_transform('base_link', point.header.frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")
            return
        
        out = do_transform_point(point, g)
        out.header.frame_id = 'base_link'
        out.header.stamp = self.get_clock().now().to_msg()

        return out

    def set_speed(self, fraction):
        """Sets the speed slider fraction."""
        if not self.speed_slider_cli.service_is_ready():
            self.get_logger().warn("Speed slider service is not ready")
            return

        request = SetSpeedSliderFraction.Request()
        request.speed_slider_fraction = fraction

        future = self.speed_slider_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Speed set to {fraction}")
        else:
            self.get_logger().error("Failed to set speed slider")

    def reset_speed(self):
        self.set_speed(0.3)  # Reset to default speed (0.1 or your preferred default speed)
        self.get_logger().info("Speed reset to default.")
    
    def release_time(self):
        """
        Calculates release time, given fixed self.v and distance d
        """

        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't align base.")
            return
        
        if self.cup_point is None:
            self.get_logger().error("No cup position available! Can't align base.")
            return

        # ----------------------------------------------------
        # Calculate distance d
        # ----------------------------------------------------

        # obtain arm length from base joint to shoulder joint
        try:
            g = self.tf_buffer.lookup_transform('shoulder_joint', 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")
            return
        
        c, t = self.cup_point.point, g.transform.translation
        a = np.linalg.norm([t.x, t.y, t.z]) # arm length

        d = np.sqrt(np.linalg.norm([c.x, c.y])**2 - a**2)

        # ---------------------------------------------------
        # Calculate tool height h
        # ---------------------------------------------------
        dx = - 0.25 # account for difference between AR marker and table
        try:
            h = self.tf_buffer.lookup_transform('tool0', 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")
            return
        # took h(tool0<-base) @ point, then took negation to find z of point wrt tool0
        h = - do_transform_point(self.cup_point, h).point.z + dx

        # ---------------------------------------------------
        # Calculate release angle (dtheta)
        # ---------------------------------------------------

        def first_bounce(theta):
            e, mu, g = 0.75, 1.0, 9.81
            vx, vz = self.v*np.cos(theta), self.v*np.sin(theta)
            t0, t1 = (vz + np.sqrt(vz**2 + 2*g*h))/g, 2*e*np.sqrt(vz**2 + 2*g*h)/g
            return vx*t0 + (mu*vx)*t1
        
        dtheta = brentq(lambda theta: first_bounce(theta) - d,
                        xtol=1e-8, ytol=1e-8, maxiter=200)

        # ---------------------------------------------------
        # Calculate release time
        # ---------------------------------------------------
        t_align = 3.2 # base alignment time takes around 3 seconds
        try:
            trans = self.tf_buffer_lookup_transform('tool0', 'base_link', rclpy.time.Time()).transform.translation
        except TransformException as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")
            return
        l = np.linalg.norm([trans.x, trans.y, trans.z]) # arm length
        dt = (np.pi/6 + dtheta) * l/self.v

        return float(t_align + dt)

def main(args=None):
    rclpy.init(args=args)
    node = UR7e_Throw()
    rclpy.spin(node)
    node.ik_planner.destroy_node()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
