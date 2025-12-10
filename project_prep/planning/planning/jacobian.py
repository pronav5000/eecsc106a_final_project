import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
import sys


class JacobianPlanner(Node):
    def __init__(self):
        super().__init__('jacobian_planner')
        self.robot_state = RobotState()

    def compute_jacobian(self, joint_state):
        robot_state = RobotState(self.robot.get_robot_model())
        robot_state.set_joint_group_positions("manipulator", joint_state)

        reference_point_position = np.array([0,0,0])
        link_name = "wrist3_link"
        jacobian = robot_state.get_jacobian(
            joint_model_group_name="ur_manipulator",
            link_name=link_name,
            reference_point_position=reference_point_position
        )       
        return jacobian

def main(args=None):
    rclpy.init(args=args)
    node = JacobianPlanner()

    # ---------- Test setup ----------
    current_state = JointState()
    current_state.name = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]
    current_state.position = [4.722, -1.850, -1.425, -1.405, 1.593, -3.141]

    # ---------- Test desired end-effector velocity ----------
    linear_velocity = [0.1, 0.0, 0.0]  # Desired velocity in x-direction (m/s)
    angular_velocity = [0.0, 0.0, 0.0]  # No angular velocity (rad/s)

    # Compute joint velocities for the desired end-effector velocity
    joint_velocities = node.compute_joint_velocities(current_state, linear_velocity, angular_velocity)

    node.get_logger().info(f"Computed joint velocities: {joint_velocities}")

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
