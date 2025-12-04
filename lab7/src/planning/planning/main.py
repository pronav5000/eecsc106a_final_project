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

from planning.ik import IKPlanner

class UR7e_CubeGrasp(Node):
    def __init__(self):
        super().__init__('cube_grasp')

        self.cube_pub = self.create_subscription(PointStamped, '/cube_pose_base', self.cube_callback, 1) # TODO: CHECK IF TOPIC ALIGNS WITH YOURS
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)
        self.cup_sub = self.create_subscription(PointStamped, '/cup_pose_base', self.cup_callback, 1)

        self.exec_ac = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_cli = self.create_client(Trigger, '/toggle_gripper')

        self.cube_pose = None
        self.cup_pose = None
        self.current_plan = None
        self.joint_state = None

        self.ik_planner = IKPlanner()

        self.job_queue = [] # Entries should be of type either JointState or String('toggle_grip')

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
        x = cube_pose.point.x
        y = cube_pose.point.y - 0.035
        z = cube_pose.point.z + 0.185
        state_1 = self.ik_planner.compute_ik(self.joint_state, x, y, z)

        self.job_queue.append(state_1)

        # 2) Move to Grasp Position (lower the gripper to the cube)
        '''
        Note that this will again be defined relative to the cube pose. 
        DO NOT CHANGE z offset lower than +0.16. 
        '''
        x = cube_pose.point.x
        y = cube_pose.point.y - 0.027
        z = cube_pose.point.z + 0.16
        state_2 = self.ik_planner.compute_ik(self.joint_state, x, y, z)
        self.job_queue.append(state_2)

        # 3) Close the gripper. See job_queue entries defined in init above for how to add this action.
        self.job_queue.append('toggle_grip')
        
        # 4) Move back to Pre-Grasp Position
        x = cube_pose.point.x
        y = cube_pose.point.y - 0.035
        z = cube_pose.point.z + 0.185
        state_3 = self.ik_planner.compute_ik(self.joint_state, x, y, z)
        self.job_queue.append(state_3)

        # 5) TODO: Construct desired joint state (use pose of the cup)
        #TODO: when we add the job, make sure to add velocities to the execute_jobs function
        #TODO: manually compute the desired velocities to get to the cup
        #TODO: remove delay and execute a gripper release before last joint state trajectory is computed
        # Add Joint Trajectory to queue

        x = cup_pose.point.x + 0.4
        y = cup_pose.point.y 
        z = cup_pose.point.z + 0.18
        self.job_queue.append(['throw_ball', x, y, z])

        # 6) Release the gripper
        self.job_queue.append('toggle_grip')

        self.execute_jobs()




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
        elif isinstance(next_job, list):
            self.get_logger().info("Throwing Ball")
            self._throw_ball(next_job)
        else:
            self.get_logger().error("Unknown job type.")
            self.execute_jobs()  # Proceed to next job

    def _throw_ball(self, job):
        cmd, x, y, z = job

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0   # facing forward, fill in as needed

        joint_goal = self.ik_planner.compute_ik(target_pose)
        if joint_goal is None:
            self.get_logger().error("IK failed for throw pose")
            return

        # 3) Compute a fast trajectory (throw speed)
        traj = self.ik_planner.plan_to_joints(joint_goal, speed_scale=1.5)
        if traj is None:
            self.get_logger().error("Failed to plan throw trajectory")
            return

        jt = traj.joint_trajectory

        # 4) MODIFY trajectory velocities to throw harder
        for p in jt.points:
            # increase velocities dramatically
            p.velocities = [v * 3.0 for v in p.velocities]  
            # optionally adjust accelerations too
            p.accelerations = [a * 2.0 for a in p.accelerations]

        # 5) Find the time to release the gripper (mid trajectory)
        total_time = jt.points[-1].time_from_start.sec \
                + jt.points[-1].time_from_start.nanosec * 1e-9

        release_time = total_time * 0.7   # release 70% through the motion

        self.get_logger().info(f"Scheduled gripper release at t={release_time:.2f}s")

        # 6) Start executing the trajectory in a thread so we can release mid-movement
        threading.Thread(target=self._execute_throw_with_release,
                        args=(jt, release_time),
                        daemon=True).start()

    def _execute_throw_with_release(self, joint_traj, release_time):
        self._execute_joint_trajectory(joint_traj)

        start = rclpy.time.Time()

        # Wait until the proper moment in the trajectory
        while rclpy.time.Time() - start < release_time:
            time.sleep(0.01)

        # Release the gripper WHILE arm is still moving
        self._toggle_gripper()
        self.get_logger().info("GRIPPER RELEASED MID-THROW")

        # Wait for trajectory to finish before continuing job queue
        self._wait_for_trajectory_done()

        # Continue with next job
        self.execute_jobs()

        
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
