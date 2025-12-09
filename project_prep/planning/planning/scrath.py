# self.job_queue.append('rotate_gripper')

#         # 6) Move to pre throw
#         '''
#         We want the release position to be 0.4m on the other side of the aruco tag relative to initial cube pose.
#         Which offset will you change to achieve this and in what direction?
#         '''        
#         joint_sol_4 = self.ik_planner.compute_ik(self.joint_state, self.cube_pose.point.x + offset_x, self.cup_pose.point.y + offset_y, self.cube_pose.point.z + offset_z + 0.8)
#         self.job_queue.append(joint_sol_4)
#         if joint_sol_4 is not None:
#             self.get_logger().info("Joint solution 4 computed succesfully.")

#         self.execute_jobs()

#         # 7) Thow by publishing to joint states topic
#         new_positions = self.joint_state.position
#         new_positions[5] = new_positions[5] + np.pi/2
#         traj = JointTrajectory()
#         traj.joint_names = self.joint_names
#         point = JointTrajectoryPoint()
#         point.positions = new_positions
#         point.velocities = [0.0]*6
#         point.velocities[5] = 1.0
#         point.time_from_start.sec = 5 # set to 5 acc to pdf
#         traj.points.append(point)
#         self.joint_pub.publish(traj)


#  def execute_jobs(self):
#         if not self.job_queue:
#             self.get_logger().info("All jobs completed.")
#             rclpy.shutdown()
#             return

#         self.get_logger().info(f"Executing job queue, {len(self.job_queue)} jobs remaining.")
#         next_job = self.job_queue.pop(0)

#         if isinstance(next_job, JointState):

#             traj = self.ik_planner.plan_to_joints(next_job)
#             if traj is None:
#                 self.get_logger().error("Failed to plan to position")
#                 return

#             self.get_logger().info("Planned to position")

#             self._execute_joint_trajectory(traj.joint_trajectory)
#         elif next_job == 'toggle_grip':
#             self.get_logger().info("Toggling gripper")
#             self._toggle_gripper()
        
#         elif next_job == 'rotate_gripper':
#             self.get_logger().info("Toggling gripper")
#             self._rotate_gripper()

#         else:
#             self.get_logger().error("Unknown job type.")
#             self.execute_jobs()  # Proceed to next job

# def _rotate_gripper(self, angle_rad=np.pi/2):
#         """
#         Rotate the wrist (last joint) by angle_rad (default +90 degrees)
#         based on the current joint state, then plan and execute.
#         """
#         if self.joint_state is None:
#             self.get_logger().error("No joint state available, cannot rotate gripper")
#             self.execute_jobs()
#             return

#         # Copy current joint state into a new target JointState
#         target = JointState()
#         target.header = self.joint_state.header
#         target.name = list(self.joint_state.name)
#         target.position = list(self.joint_state.position)

#         # Find wrist_3 joint index (or fall back to last joint)
#         try:
#             wrist_idx = target.name.index('wrist_3_joint')
#         except ValueError:
#             # If names are different in your URDF, either adjust this string
#             # or just use the last joint as a fallback
#             wrist_idx = len(target.position) - 1
#             self.get_logger().warn(
#                 f"'wrist_3_joint' not found, using joint index {wrist_idx} as wrist."
#             )

#         # Add +90 degrees (or whatever angle), in radians
#         target.position[wrist_idx] += angle_rad

#         # Plan to this new joint configuration
#         traj = self.ik_planner.plan_to_joints(target)
#         if traj is None:
#             self.get_logger().error("Failed to plan gripper rotation")
#             self.execute_jobs()
#             return

#         self.get_logger().info("Planned gripper rotation, executing...")
#         self._execute_joint_trajectory(traj.joint_trajectory)
