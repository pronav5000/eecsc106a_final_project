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

from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.cube_pub = self.create_subscription(PointStamped, '/ball_pos', self.cube_callback, 1) # TODO: CHECK IF TOPIC ALIGNS WITH YOURS
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        self.joint_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.current_plan = None
        self.joint_state = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.ik_planner = IKPlanner()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

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

        # The transformed point is now in the base_link frame
        transformed_position = transformed_point.point
        self.cube_pose.point = transformed_position
        self.get_logger().info(f"Transformed ball position in base frame: X={transformed_position.x} Y={transformed_position.y} Z={transformed_position.z}")

            # Use the transformed position to compute IK
        # -----------------------------------------------------------
        # TODO: In the following section you will add joint angles to the job queue. 
        # Entries of the job queue should be of type either JointState or String('toggle_grip')
        # Think about you will leverage the IK planner to get joint configurations for the cube grasping task.
        # To understand how the queue works, refer to the execute_jobs() function below.
        # -----------------------------------------------------------

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

        offset_x, offset_y, offset_z = 0.0, -0.035, 0.185

        joint_sol_1 = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + offset_x, self.cube_pose.point.y + offset_y, self.cube_pose.point.z + offset_z)
        self.job_queue.append(joint_sol_1)
        if joint_sol_1 is not None:
            self.get_logger().info("Joint solution 1 computed succesfully.")

        # 2) Move to Grasp Position (lower the gripper to the cube)
        '''
        Note that this will again be defined relative to the cube pose. 
        DO NOT CHANGE z offset lower than +0.16. 
        '''
        joint_sol_2 = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + offset_x, self.cube_pose.point.y + offset_y -0.01, self.cube_pose.point.z + offset_z -0.02)
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


        # 5) Move to release Position
        '''
        We want the release position to be 0.4m on the other side of the aruco tag relative to initial cube pose.
        Which offset will you change to achieve this and in what direction?
        '''        
        joint_sol_4 = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + offset_x, self.cube_pose.point.y + offset_y, self.cube_pose.point.z + offset_z + 0.5)
        self.job_queue.append(joint_sol_4)
        if joint_sol_4 is not None:
            self.get_logger().info("Joint solution 4 computed succesfully.")
        
        self.job_queue.append('rotate_gripper')

        # # 6) Release the gripper
        # self.job_queue.append('toggle_grip')
        # print('Job Queue', self.job_queue)

        # 7) Thow by publishing to joint states topic

        self.job_queue.append('throw_ball')

        self.execute_jobs()
    
    # [OLD] def throw_ball(self):
    #     new_positions = list(self.joint_state.position)
    #     self.get_logger().info(f"new_positions:  {new_positions}")

    #     new_positions[4] = new_positions[4] + np.pi/2
    #     traj = JointTrajectory()
    #     traj.joint_names = self.joint_names
    #     point = JointTrajectoryPoint()
    #     point.positions = new_positions
    #     point.velocities = [0.0]*6
    #     point.velocities[4] = 1.0
    #     point.time_from_start.sec = 5 # set to 5 acc to pdf

    #     self.get_logger().info(f"new_positions after velocities set to 1.0: {new_positions}")
    #     traj.points.append(point)
    #     self.joint_pub.publish(traj)
    # def throw_ball(self,
    #            throwing_joint_index = 4,      # index in self.joint_names to use for throw
    #            throw_angle=np.pi/2,        # radians to move (e.g. 90 degrees)
    #            throw_duration=0.3):        # seconds for the flick

    #     # 1) Make sure we have a current joint state
    #     if self.joint_state is None:
    #         self.get_logger().error("No joint state available! Can't throw.")
    #         return

    #     # 2) Read current joint positions as pre-throw configuration
    #     start_positions = list(self.joint_state.position)

    #     if throwing_joint_index < 0 or throwing_joint_index >= len(start_positions):
    #         self.get_logger().error(f"Invalid throwing_joint_index: {throwing_joint_index}")
    #         return

    #     start_angle = start_positions[throwing_joint_index]
    #     final_angle = start_angle + throw_angle

    #     # 3) Build final positions: only one joint changes
    #     final_positions = start_positions.copy()
    #     final_positions[throwing_joint_index] = final_angle

    #     # 4) Compute nominal joint velocity for this motion
    #     #    (constant-velocity assumption over throw_duration)
    #     if throw_duration <= 0.0:
    #         self.get_logger().error("throw_duration must be > 0")
    #         return

    #     joint_velocity = throw_angle / throw_duration  # rad/s for that joint

    #     # 5) Build the trajectory message
    #     traj = JointTrajectory()
    #     traj.joint_names = self.joint_names

    #     # Point 0: current pose, zero velocity
    #     pt0 = JointTrajectoryPoint()
    #     pt0.positions = start_positions
    #     pt0.velocities = [0.0] * len(self.joint_names)
    #     pt0.time_from_start.sec = 0
    #     pt0.time_from_start.nanosec = 0

    #     # Point 1: final pose, with desired velocity on throwing joint
    #     pt1 = JointTrajectoryPoint()
    #     pt1.positions = final_positions
    #     pt1.velocities = [0.0] * len(self.joint_names)
    #     pt1.velocities[throwing_joint_index] = joint_velocity
    #     pt1.time_from_start.sec = int(throw_duration)
    #     pt1.time_from_start.nanosec = int((throw_duration - int(throw_duration)) * 1e9)

    #     traj.points.append(pt0)
    #     traj.points.append(pt1)

    #     self.get_logger().info(
    #         f"Throwing with joint {throwing_joint_index}: "
    #         f"{start_angle:.3f} -> {final_angle:.3f} rad in {throw_duration:.3f} s "
    #         f"(vel ~ {joint_velocity:.3f} rad/s)"
    #     )

    #     # 6) Publish to the joint trajectory controller
    #     # self.joint_pub.publish(traj)
    #     self._execute_joint_trajectory(traj)
    def throw_ball(self,
               throw_angle=np.pi/2,
               throw_velocity=1.0):

        # ---------------------------------------------
        # 1) MUST have the current joint state
        # ---------------------------------------------
        if self.joint_state is None:
            self.get_logger().error("No joint state available! Can't throw.")
            return

        # ---------------------------------------------
        # 2) Call IK to get joint positions for the pose
        # ---------------------------------------------
        target = JointState()
        target.header = self.joint_state.header
        target.name = list(self.joint_state.name)
        target.position = list(self.joint_state.position)
        target.position[3] = target.position[3] + np.pi/2

        traj = self.ik_planner.plan_to_joints(target)
        self._execute_joint_trajectory(traj.joint_trajectory)

        # try:
        #     ik_result = self.ik_client.call(ik_req)
        # except Exception as e:
        #     self.get_logger().error(f"IK call failed: {e}")
        #     return

        # if not ik_result.solution.joint_state.position:
        #     self.get_logger().error("IK returned no solution.")
        #     return

        # ik_joints = list(ik_result.solution.joint_state.position)

        # # ---------------------------------------------
        # # 3) Compute final throwing joint angle
        # # ---------------------------------------------
        # throwing_joint = 4   # <-- per your instruction

        # start_angle = ik_joints[throwing_joint]
        # end_angle = start_angle + throw_angle

        # # Replace only joint 4 with the throw motion
        # final_joints = ik_joints.copy()
        # final_joints[throwing_joint] = end_angle

        # # ---------------------------------------------
        # # 4) Compute duration from velocity
        # # ---------------------------------------------
        # duration = abs(throw_angle / throw_velocity)

        # # ---------------------------------------------
        # # 5) Build trajectory with TWO points
        # # ---------------------------------------------
        # traj = JointTrajectory()
        # traj.joint_names = self.joint_names   # MUST match controller exactly

        # # ---- point 0: start state (IK solution before throw) ----
        # pt0 = JointTrajectoryPoint()
        # pt0.positions = ik_joints
        # pt0.velocities = [0.0] * len(self.joint_names)
        # pt0.time_from_start.sec = 0
        # pt0.time_from_start.nanosec = 0

        # # ---- point 1: end state (after throw) ----
        # pt1 = JointTrajectoryPoint()
        # pt1.positions = final_joints
        # pt1.velocities = [0.0] * len(self.joint_names)   # stop at end
        # pt1.time_from_start.sec = int(duration)
        # pt1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        # traj.points.append(pt0)
        # traj.points.append(pt1)

        # # ---------------------------------------------
        # # 6) Send trajectory
        # # ---------------------------------------------
        # self.get_logger().info(
        #     f"Throwing with joint 4 from {start_angle:.2f} → {end_angle:.2f} rad "
        #     f"in {duration:.2f} s"
        # )

        # self.joint_pub.publish(traj)





    # def send_single_joint_motion(self, joint_index: int, delta_rad: float, duration: float = 2.0):
    #     # Wait until we have a joint state
    #     if self.joint_state is None:
    #         self.get_logger().warn("No joint state yet, cannot send motion")
    #         return

    #     # Map joint_state positions to our ordered joint_names
    #     current_positions = [0.0] * len(self.joint_names)
    #     for i, name in enumerate(self.joint_names):
    #         try:
    #             idx = self._joint_state.name.index(name)
    #             current_positions[i] = self._joint_state.position[idx]
    #         except ValueError:
    #             self.get_logger().error(f"Joint {name} not found in joint_states!")
    #             return

    #     # Build trajectory
    #     traj = JointTrajectory()
    #     traj.joint_names = self.joint_names

    #     # Start point: current pose
    #     p0 = JointTrajectoryPoint()
    #     p0.positions = list(current_positions)
    #     p0.time_from_start.sec = 0

    #     # End point: same as current, but with one joint changed
    #     p1 = JointTrajectoryPoint()
    #     p1.positions = list(current_positions)
    #     p1.positions[joint_index] += delta_rad
    #     p1.time_from_start.sec = int(duration)

    #     traj.points = [p0, p1]

    #     # Send as FollowJointTrajectory goal
    #     goal = FollowJointTrajectory.Goal()
    #     goal.trajectory = traj

    #     self._ac.wait_for_server()
    #     self.get_logger().info(f"Sending single-joint motion on index {joint_index}")
    #     future = self._ac.send_goal_async(goal)
    #     # you can attach callbacks for result if you like


        


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
