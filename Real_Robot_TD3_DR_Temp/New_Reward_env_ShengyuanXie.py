import numpy as np
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from tf.transformations import euler_from_quaternion
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
INIT = [-0.022, -1.582, -1.561, -1.624, -1.599, 0.0]
##INIT = [-0.218, -1.512, -1.922, -0.588, -1.527, 0.010]
#INIT = [0.092, -1.927, -2.239, -0.613, 1.589, 1.833]
#GOAL = [0.282, -2.056, -2.150, -0.876, -1.609, 0.0]
#INIT = [0.018, -0.267, 0.262, 0.262, -0.262, -0.000]
#INIT = [0.149, -1.932, -1.821, -1.763, 1.463, 0.009]
GOAL = [-0.49079821 , 0.0460181   ,0.13643442, -0.49079821  ,0.0460181  , 0.13643442]

#final end effector position
#GOAL = [-0.44928456, 0.06760729,  0.06409959,  3.13817389, -0.06887363, -0.17547368]

class Ur5():
    def __init__(self, init_joints=INIT, goal_pose=GOAL, duration=0.01):
        rospy.init_node('ur5_env', anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")
        self.current_joints = init_joints
        self.duration = duration
        self.state_dim = 11
        self.action_dim = 5
        self.get_counter = 0  # Counter for success checks
        self.goal_pose = np.array(goal_pose)
    
    def change_signs_of_action(self, action):
        for i in range(len(action)):
            if action[i] < 0:
                action[i] = abs(action[i])
            else:
                if action[i] > 0:
                    action[i] = -(action[i])
        return action

    def step(self, action, max_attempts=5):
        # Convert numpy array to Python list with standard float
        action = action.tolist()

        # UR5的关节限制 (以弧度为单位)
        joint_limits = [
            (-np.pi, np.pi),  # shoulder_pan_joint
            (-np.pi, np.pi),  # shoulder_lift_joint
            (-np.pi, np.pi),  # elbow_joint
            (-np.pi, np.pi),  # wrist_1_joint
            (-np.pi, np.pi),  # wrist_2_joint
            (-np.pi, np.pi)   # wrist_3_joint
        ]

        # 如果DDPG只生成了5个关节的动作，为第6个关节设置默认值
        if len(action) == 5:
            action.append(0.0)  # 例如，默认值设置为0.0或保持不变

        attempt = 0
        while attempt < max_attempts:
            out_of_bounds = False
            # 检查并限制每个关节的角度
            for i, (low, high) in enumerate(joint_limits):
                if action[i] < low:
                    action[i] = low
                    out_of_bounds = True
                elif action[i] > high:
                    action[i] = high
                    out_of_bounds = True
            
            action[2] = 0.9 * action[2]
            action[3] = 0.5 * (action[3] - np.pi)
            action[4] = 0.7 * action[4]

            if not out_of_bounds:
                # 如果动作在允许范围内，执行动作
                #set_action_value = self.change_signs_of_action(action)
                self.group.set_joint_value_target(action)
                self.group.go(wait=True)
                self.group.stop()

                position, rpy = self.get_pos()
                state = self.get_state(action, position, rpy)
                reward, terminal, target_pos, end_eff_pos = self.get_reward(position, rpy)
                terminal = False
                return state, reward, terminal, target_pos, end_eff_pos

            # 如果超出范围，重新生成动作
            action = self.regenerate_action()
            if len(action) == 5:
                action.append(0.0)  # 确保为第6个关节设置默认值
            attempt += 1

        # 如果超过最大尝试次数，返回原始状态并给出负奖励
        #position = self.get_pos()[0]
        #state = self.get_state([0, 0, 0, 0, 0, 0], position)
        #reward = -1
        #terminal = True
        return state, reward, terminal, target_pos, position

    def regenerate_action(self):
        # 使用DDPG重新生成一个动作
        state = self.get_state(self.current_joints, self.get_pos()[0])
        action = model.choose_action(state)  # 需要传递模型实例
        return action.tolist()
    
    ###Resetting function for Joint angles
    ###with the current value
    def reset(self):
        self.group.set_joint_value_target(self.current_joints)
        self.group.go(wait=True)
        self.group.stop()
        position, rpy = self.get_pos()
        print("The position is: ", position)
        print("The orinetation is: ",rpy)

        ####State has 5 actions, and position having (x, y, z) coordinates
        return self.get_state([0, 0, 0, 0, 0], position, rpy)

    def get_pos(self):
        
        # Get the current joint states
        #joint_state = rospy.wait_for_message('/joint_states', JointState, timeout=1.0)
        #current_joints = joint_state.position

        # Get the current end-effector pose
        current_pose = self.group.get_current_pose().pose

        # Print for debugging
        #rospy.loginfo(f"Current joints: {current_joints}")
        #rospy.loginfo(f"Current pose: {current_pose}")

        position = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        rpy = euler_from_quaternion(orientation)

        return position, np.array(rpy)
        

    def get_state(self,action, position,rpy):
        #x, y, z of angular distance
        goal_pose = self.goal_pose[:3]

        #x,y, z of angualr orientation
        goal_orient = self.goal_pose[3:]

        #print("Shape of rpy:", np.shape(rpy))
        
        #goal_dis = np.linalg.norm(goal_pose-self.base_pos)
        #print(rpy[0])
        #print(position)
        
        ####pose_6 estimates the angular distance between the goal pose and current positiuon
        pose_6 = position - goal_pose
        orient_6 = rpy - goal_orient
        ###pose_6 must be absplute value
        

        #dis_6 = np.linalg.norm(pose_6)
        ###absolute value

        #in_point = 1 if self.get_counter > 0 else 0
        if len(action) == 6:
            action = action[0:5]
        
        state = np.concatenate((action, pose_6, orient_6),axis=None)
        #state = np.concatenate((pose_6,goal_dis,action,in_point),axis=None)
        #state = state / np.linalg.norm(state)

        return state

    def get_reward(self,pos,rpy):
        t = False
        #Compute reward based on angular distance
        dis = np.linalg.norm(self.goal_pose[:3]-pos)
        #add regularization term

        #compute orientation distance
        dis_a = np.linalg.norm(self.goal_pose[3:]-rpy)
        # print(self.goal_pose[3])
        # print(rpy[0])
        #r_a = -0.5 * dis_a

        reward = - dis - 0.1 * dis_a

        collision_coeff = 0

        ####If Collisiion occurs
        if dis == 0.1 and dis_a == 0.1:
            collision_coeff = 1
            reward  = reward - collision_coeff

        if self.get_counter < 30:
            reward = reward - collision_coeff
            if dis < 0.1:
                reward += 1
                print ('reach distance')
                if dis_a < 0.1:
                    reward += 2
                    ####setting the flag to true
                    ###to signify succesfully reaching the terminal
                    t = True
                    print ('reach rotation')
                    print('successfully complete task')
                    print('############################')
        else:
            if self.get_counter == 30:
                ###only reaching the distance
                if dis < 0.1:
                    reward += 1
                    print ('reach distance')
                    ####Checking if the orientation is reached
                    if dis_a < 0.1:
                        print ('reach rotation')
                        print('successfully complete task')
                        print('############################')
                        reward += 10
                        t = True
                else:
                    reward += 0

        self.get_counter += 1
        
        return reward, t, self.goal_pose[:3], pos

if __name__ == '__main__':
    arm = Ur5()
