import numpy as np
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from tf.transformations import euler_from_quaternion

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
INIT = [0.0, -1.92, 2.0, -1.68, -1.56, 0.0]

class Ur5():
    def __init__(self, init_joints=INIT, duration=0.01):
        rospy.init_node('ur5_env', anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")
        self.current_joints = init_joints
        self.duration = duration
        self.state_dim = 10
        self.action_dim = 5
        self.get_counter = 0  # Counter for success checks

    def step(self, action):
        # Apply DDPG generated joint angles directly to the robot
        self.group.set_joint_value_target(action)
        self.group.go(wait=True)
        self.group.stop()

        # Get the current position and orientation of the end-effector
        position, rpy = self.get_pos()
        state = self.get_state(action, position)
        reward = self.get_reward(position, rpy, action)
        terminal = False  # No specific termination condition here
        return state, reward, terminal

    def reset(self):
        self.group.set_joint_value_target(self.current_joints)
        self.group.go(wait=True)
        self.group.stop()
        position, _ = self.get_pos()
        return self.get_state([0, 0, 0, 0, 0], position)

    def get_pos(self):
        current_pose = self.group.get_current_pose().pose
        position = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        rpy = euler_from_quaternion(orientation)
        return position, np.array(rpy)

    def get_state(self, action, position):
        # In this scenario, the state might only include the action (joint angles) and other features you want to monitor.
        state = np.concatenate((position, action), axis=None)
        return state

    def get_reward(self, pos, rpy, action):
        # Reward calculation can be simplified or even skipped if not needed
        reward = -0.01 * np.linalg.norm(action)  # Regularization term
        return reward

if __name__ == '__main__':
    arm = Ur5()
