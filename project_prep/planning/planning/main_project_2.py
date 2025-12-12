# ROS Libraries
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped 
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import numpy as np
import tf2_geometry_msgs
from tf2_ros import TransformException
from ur_msgs.srv import SetSpeedSliderFraction
import time


from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')
        

        self.cube_pub = self.create_subscription(PointStamped, '/ball_pos', self.cube_callback, 1) 
        self.cube_sub = self.create_subscription(PointStamped, '/cup_point', self.cup_callback, 1) 
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.joint_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.speed_slider_cli = self.create_client(
            SetSpeedSliderFraction, '/io_and_status_controller/set_speed_slider'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.cup_pose = None
        self.current_plan = None
        self.joint_state = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.ik_planner = IKPlanner()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.job_queue = [] 
   

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg
    
    def cup_callback(self, cup_pose):
        if cup_pose is None:
            self.get_logger().info("No cu[] position HELP")
            return

        if self.cup_pose is not None:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform('base_link', cup_pose.header.frame_id, rclpy.time.Time())
        except:
            self.get_logger().info("Transform not available to look up")
            #self.cup_pose = None
            return
        
        transformed_point = tf2_geometry_msgs.do_transform_point(cup_pose, transform)
        transformed_position = transformed_point.point
        self.cup_pose = cup_pose
        self.cup_pose.point = transformed_position
        self.cup_pose.header.frame_id = "base_link"
        self.cup_pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Transformed cup position in base frame: X={transformed_position.x} Y={transformed_position.y} Z={transformed_position.z}")

    def cube_callback(self, cube_pose):
        if cube_pose is None:
            self.get_logger().info("No cube position HELP")
            return

        if self.cube_pose is not None:
            return

        if self.joint_state is None:
            self.get_logger().info("No joint state yet, cannot proceed")
            return

        self.cube_pose = cube_pose

        try:
            transform = self.tf_buffer.lookup_transform('base_link', cube_pose.header.frame_id, rclpy.time.Time())
        except:
            self.get_logger().info("Transform not available to look up")
            self.cube_pose = None
            return
        
        transformed_point = tf2_geometry_msgs.do_transform_point(cube_pose, transform)
        transformed_position = transformed_point.point
        self.cube_pose.point = transformed_position
        self.get_logger().info(f"Transformed ball position in base frame: X={transformed_position.x} Y={transformed_position.y} Z={transformed_position.z}")

        if self.cup_pose is None:
            self.get_logger().info("No cu[] position HELP")
            return

        # 1) Move to Pre-Grasp Position (gripper above the cube)
        '''
        Use the following offsets for pre-grasp position:
        x offset: 0.0
        y offset: -0.035 (Think back to lab 5, why is this needed?)
        z offset: +0.185 (to be above the cube by accounting for gripper length)
        '''
        self.get_logger().info("Starting joint solution calculation")
        self.get_logger().info(f"x: {self.cube_pose.point.x}")
        self.get_logger().info(f"y: {self.cube_pose.point.y}")
        self.get_logger().info(f"z: {self.cube_pose.point.z}")

        # offset_x, offset_y, offset_z = 0.018, -0.02, 0.25
        offset_x, offset_y, offset_z = 0.018, -0.02, 0.25

        # 1) Move to ball

        joint_sol_1 = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + offset_x, self.cube_pose.point.y + offset_y, self.cube_pose.point.z + offset_z)
        self.job_queue.append(joint_sol_1)
        if joint_sol_1 is not None:
            self.get_logger().info("Joint solution 1 computed succesfully.")

        # 2) Move to Grasp Position (lower the gripper to the cube) (Do not change z offset lower than +0.16)
        '''
        Note that this will again be defined relative to the cube pose. 
        DO NOT CHANGE z offset lower than +0.16. 
        '''
        joint_sol_2 = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + offset_x, self.cube_pose.point.y + offset_y, self.cube_pose.point.z + offset_z -0.045)
        self.job_queue.append(joint_sol_2)
        if joint_sol_2 is not None:
            self.get_logger().info("Joint solution 2 computed succesfully.")

        # 3) Close the gripper. See job_queue entries defined in init above for how to add this action.
        self.job_queue.append('toggle_grip')
        
        # 4) Move back to Pre-Grasp Position
        joint_sol_3 = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + offset_x, self.cube_pose.point.y + offset_y, self.cube_pose.point.z + offset_z + 0.1)
        self.job_queue.append(joint_sol_3)
        if joint_sol_3 is not None:
            self.get_logger().info("Joint solution 3 computed succesfully.")


        # 5) Move to tucked Position
        self.job_queue.append('wind_up')

        # 6) Move to align position
        self.job_queue.append('align_base')
        # joint_sol_4 = self.ik_planner.compute_ik(self.joint_state, self.cup_pose.point.x + offset_x, self.cube_pose.point.y + offset_y, self.cube_pose.point.z + offset_z, qx=0.0, qy=1.0, qz=0.0, qw=0.0)
        # self.get_logger().info(f"{joint_sol_4}")
        # self.job_queue.append(joint_sol_4)
        # if joint_sol_4 is not None:
        #     self.get_logger().info("Joint solution 4 computed succesfully.")        
        
        # 7) Move to shoot position
        self.job_queue.append('thow_ball')
        self.execute_jobs()
   
    # def align_base(self):
    #     if self.joint_state is None:
    #         self.get_logger().error("No joint state available! Can't throw.")
    #         return

    #     transform = self.tf_buffer.lookup_transform('wrist_3_link', self.cup_pose.header.frame_id, rclpy.time.Time())
    #     transformed_point = tf2_geometry_msgs.do_transform_point(self.cup_pose, transform)
    #     transformed_position = transformed_point.point
        
    #     # base_joint_angle = 5.293953895568848
    #     offset_shoulder_pan = 0 # TODO: Fix offset
    #     # base_joint_angle = np.arctan2(transformed_position.y, transformed_position.x)
    #     target = JointState()
    #     target.header = self.joint_state.header
    #     target.name = list(self.joint_state.name)
    #     target.position = list(self.joint_state.position)
    #     target.position[5] = target.position[5] + offset_shoulder_pan
    #     target.velocity = [0.0]*6
    #     target.velocity[5] = 1.0

    #     traj = self.ik_planner.plan_to_joints(target)
    #     self._execute_joint_trajectory(traj.joint_trajectory)
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

        d0_xy = self.quaternion_to_y_axis_xy(
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
            alpha = -window * (i / (num_samples - 1))
            # alpha = -window + (2.0 * window) * (i / (num_samples - 1)) - this is for two samples
            theta_candidate = theta_guess + alpha
            delta = theta_candidate - theta0

            p_e_xy = self.rotate_xy(p_e0_xy, delta)
            d_xy = self.rotate_xy(d0_xy, delta)

            n_d = np.linalg.norm(d_xy)
            if n_d < 1e-8:
                continue
            d_xy = d_xy / n_d

            err = self.line_distance_to_point(p_e_xy, d_xy, p_c_xy)

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


    def wind_up(self, joint, angle):
    
        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't throw.")
            return

        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = [-2.3977424106993617,
                           -0.9090007543563843,
                           0.179469936558564,
                           1.5789344310764694,
                           -3.14915115034921,
                           self.joint_state.position[5]] # TODO: fill with pre shoot position

        traj = self.ik_planner.plan_to_joints(target)
        self._execute_joint_trajectory(traj.joint_trajectory)
       
    def throw_ball(self):

        self.set_speed(0.7)

        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't throw.")
            return
        offset_shoulder_lift = -1.9057523212828578 - (-2.3977424106993617)
        offset_elbow = -2.3659281730651855 - (-0.9090007543563843)
        offset_wrist_1 = 0.28631338477134705 - (0.179469936558564)

        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)
        target.position[0] = target.position[0] + offset_shoulder_lift
        target.position[1] = target.position[1] + offset_elbow
        target.position[2] = target.position[2] + offset_wrist_1

        target.velocity = [0.0]*6
        target.velocity[0] = 15
        target.velocity[1] = 15
        target.velocity[2] = 15

        traj = self.ik_planner.plan_to_joints(target)
        self._execute_joint_trajectory(traj.joint_trajectory)
        self.get_logger().info("Toggling Gripper")
        self.timer = self.create_timer(0.82, self.execute_gripper_toggle)


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
        self.set_speed(0.15)  # Reset to default speed (0.1 or your preferred default speed)
        self.get_logger().info("Speed reset to default.")

    
    def restore_state(self):

        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't throw.")
            return
        
        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)
        target.position = [-1.8433028660216273, -1.433196783065796, -1.3969979894212265, 1.5829309225082397, -3.135949436818258, 4.719995021820068]
      
      

        traj = self.ik_planner.plan_to_joints(target)
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

            self._execute_joint_trajectory(traj.joint_trajectory)
        elif next_job == 'toggle_grip':
            self.get_logger().info("Toggling gripper")
            self._toggle_gripper()
        elif next_job == 'rotate_gripper':
            self.get_logger().info("Toggling gripper")
            self._rotate_gripper()
        elif next_job == 'throw_ball':
            self.get_logger().info("Throwing ball")
            self.throw_ball()
        elif next_job == 'restore_state':
            self.get_logger().info("Restoring to tuck")
            self.restore_state()
        elif next_job == 'align_base':
            self.get_logger().info("Moving to base")
            self.align_base()
        elif next_job == 'reset_speed':
            self.get_logger().info("Reset Speed")
            self.reset_speed()
        elif isinstance(next_job, list) and next_job[0] == 'wind_up':
            self.get_logger().info("Winding")
            self.wind_up(next_job[1], next_job[2])
        else:
            self.get_logger().error("Unknown job type.")
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

        # Copy current joint state into a new target JointState
        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)

        # Find wrist_3 joint index (or fall back to last joint)
        try:
            wrist_idx = target.name.index('wrist_3_joint')
        except ValueError:
            # If names are different in your URDF, either adjust this string
            # or just use the last joint as a fallback
            wrist_idx = len(target.position) - 1
            self.get_logger().warn(
                f"'wrist_3_joint' not found, using joint index {wrist_idx} as wrist."
            )

        # Add +90 degrees (or whatever angle), in radians
        target.position[wrist_idx] += angle_rad

        # Plan to this new joint configuration
        traj = self.ik_planner.plan_to_joints(target)
        if traj is None:
            self.get_logger().error("Failed to plan gripper rotation")
            self.execute_jobs()
            return

        self.get_logger().info("Planned gripper rotation, executing...")
        self._execute_joint_trajectory(traj.joint_trajectory)
      
            
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


def main(args=None):
    rclpy.init(args=args)
    node = UR7e_CubeGrasp()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
