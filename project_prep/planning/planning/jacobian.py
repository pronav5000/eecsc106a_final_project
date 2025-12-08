import rclpy
from rclpy.node import Node
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
import numpy as np
from sensor_msgs.msg import JointState
import sys

class JacobianPlanner(Node):
    def __init__(self):
        super().__init__('jacobian_planner')
        
        # ---- Initialize MoveIt ----
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("ur_manipulator")  # Replace with your actual group name

    def get_jacobian_for_current_state(self, joint_state):
        # Set the robot's current joint values
        self.group.set_joint_value_target(joint_state)
        
        # Compute the Jacobian matrix at the current joint configuration
        jacobian = self.group.get_jacobian_matrix(self.group.get_current_joint_values())
        
        return jacobian
    
    def compute_joint_velocities(self, current_joint_state, linear_velocity, angular_velocity):
        # Get the Jacobian matrix for the current joint state
        jacobian = self.get_jacobian_for_current_state(current_joint_state)
        
        # Combine the linear and angular velocities into a single vector
        end_effector_velocity = np.array([linear_velocity[0], linear_velocity[1], linear_velocity[2],
                                          angular_velocity[0], angular_velocity[1], angular_velocity[2]])
        
        # Compute joint velocities: dot(Jacobian^-1, velocity)
        joint_velocities = np.linalg.inv(jacobian) @ end_effector_velocity
        
        return joint_velocities

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
