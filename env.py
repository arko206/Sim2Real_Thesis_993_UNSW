import numpy as np 
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from tf import TransformListener
from math import pi 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sys, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
DURATION = 0.01
GOAL = [0.525000, 0.00, 0.854007, 0.00, 0.00, 0.00]
###Initial Home position of Ur5 Arm
INIT = [0.0, -1.571, 1.571, -1.571, 0.0, 0.0]


class Ur5():
    get_counter = 0
    #get_rotation = 0

    def __init__(self, init_joints=INIT, goal_pose=GOAL, duration=DURATION):
        rospy.init_node('ur5_env', anonymous=True)
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        self.client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',
                                                                FollowJointTrajectoryAction)
        self.client.wait_for_server()
        self.initial= FollowJointTrajectoryGoal()
        self.initial.trajectory = JointTrajectory()
        self.initial.trajectory.joint_names = JOINT_NAMES
        self.current_joints = init_joints
        self.initial.trajectory.points = [JointTrajectoryPoint(positions=INIT, velocities=[0]*6, 
                                                                        time_from_start=rospy.Duration(duration))]                                                                
        self.tf = TransformListener()                            
        self.goal_pose = np.array(goal_pose)
        self.base_pos = self.get_pos(link_name='base_link')
        
        self.duration = duration
        self.state_dim = 11
        self.action_dim = 5
        self.target_generate()
    
    def step(self,action):
        #Execute action 

        self.client.cancel_all_goals()

        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        #set action joint limit
        action_ = np.concatenate((action,0),axis=None)
        self.current_joints += action_
        action_sent = np.zeros(6)
        #adjust joints degree  which has bound [-pi,pi]
        for i in range(5):
            #self.current_joints[i] = self.current_joints[i] % np.pi if self.current_joints[i] > 0 elif self.current_joints[i] % -np.pi
            if self.current_joints[i] > np.pi:
                action_sent[i] = self.current_joints[i] % -np.pi
            elif self.current_joints[i] < -np.pi:
                action_sent[i] = self.current_joints[i] % np.pi
            else:
                action_sent[i] = self.current_joints[i]
        action_sent[2] = 0.9 * action_sent[2]
        action_sent[3] = 0.5 * (action_sent[3] - np.pi)
        action_sent[4] = 0.7 * action_sent[4]

        goal.trajectory.points = [JointTrajectoryPoint(positions=action_sent, velocities=[0]*6, 
                                                                    time_from_start=rospy.Duration(self.duration))]
        
        # np.savetxt('joint_angles.txt',goal,fmt='%d')
        self.client.send_goal(goal)
        self.client.wait_for_result()
        position, rpy = self.get_pos(link_name='ee_link')

        state = self.get_state(action,position, rpy)
        reward, terminal, target_pos, end_eff_pos = self.get_reward(position,rpy)

        return state, reward, terminal, target_pos, end_eff_pos

    def reset(self):
        
        ###First Cancelling all the goals
        self.client.cancel_all_goals()

        self.current_joints = INIT
        self.client.send_goal(self.initial)
        self.client.wait_for_result()
        self.target_generate()
        position, rpy = self.get_pos(link_name='ee_link')

        #print(position)
        #print("*******")
        #print(rpy)

        state_obtained = self.get_state([0,0,0,0,0],position, rpy)

        return state_obtained

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

        if self.get_counter < 200:
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
            if self.get_counter == 200:
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
        
        return reward, t, self.goal_pose[3:], pos
    
    def get_pos(self,link_name='ee_link',ref_link='world'):
        position = None
        while position is None:
            try:
                # if self.tf.frameExists('wrist_2_link') and self.tf.frameExists(link_name):
                #     t = self.tf.getLatestCommonTime(ref_link, link_name)
                if self.tf.canTransform(ref_link, link_name, rospy.Time().now()):
                    position, quaternion = self.tf.lookupTransform(ref_link, link_name, rospy.Time(0))
                    rpy = euler_from_quaternion(quaternion)
                    # print(position)
            except:
                print ('fail to get data from tf')

        return np.array(position), np.array(rpy)
        
    def target_vis(self,goal):
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        
        s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        
        orient = Quaternion(*tf.transformations.quaternion_from_euler(1.571, 0, 0))
        origin_pose = Pose(Point(goal[0],goal[1],goal[2]), orient)
        
        #with open('/home/robocupathome/arkaur5_ws/src/contexualaffordance/models/models/table/model.sdf', "r") as f:
             #table_xml = f.read()

        #table_name = "table"
        #table_pose = Pose(Point(goal[0],goal[1],goal[2]), orient)  # Define the desired position and orientation for the block
        #print(table_xml)
        #print(table_pose)
        #delete_model(table_name)
        #s(table_name, table_xml, "", table_pose, "world")
        
        


        with open('/home/robocupathome/arkaur5_ws/src/contexualaffordance/models/box_red/model.sdf',"r") as f:
            reel_xml = f.read()
        
        for row in [1]:
            for col in range(1):
                reel_name = "reel_%d_%d" % (row,col)
                delete_model(reel_name)
                pose = deepcopy(origin_pose)
                pose.position.x = origin_pose.position.x + 0.02
                if pose.position.x > 3:
                  pose.position.x = 0.506
                pose.position.y = origin_pose.position.y + 0
                pose.position.z = origin_pose.position.z + 0
                s(reel_name, reel_xml, "", pose, "world")
                # print("spawnobj")
        
    def target_generate(self):
        rand_x, rand_y, rand_z= np.random.uniform(-0.10,0.10), np.random.uniform(-0.3,0.1), np.random.uniform(-0.6,0)

        # get_target_pose
        self.goal_pose = np.array(GOAL)
        self.goal_pose[0] += rand_x
        self.goal_pose[1] += 0
        self.goal_pose[2] += 0
        self.target_vis(self.goal_pose)

    def uniform_exploration(self, action):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES
        #add wrist_3_joint with 0 to the action
        action_ = np.concatenate((action,0),axis=None)
        self.current_joints += action_
        action_sent = np.zeros(6)
        #adjust joints degree  which has bound [-pi,pi]
        for i in range(5):
            if self.current_joints[i] > np.pi:
                action_sent[i] = self.current_joints[i] % -np.pi
            elif self.current_joints[i] < -np.pi:
                action_sent[i] = self.current_joints[i] % np.pi
            else:
                action_sent[i] = self.current_joints[i]
        #set constraint for joints
        #elbow_joint: [-0.7*pi,0.7*pi]
        action_sent[2] = 0.7 * action_sent[2]
        #wrist_3_joint: [-pi,0]
        action_sent[3] = 0.5 * (action_sent[3] - np.pi)
        #action_sent[4] = 0.7 * action_sent[4]
        #sent joint to controller
        goal.trajectory.points = [JointTrajectoryPoint(positions=action_sent, velocities=[0]*6, 
                                                                    time_from_start=rospy.Duration(self.duration))]
        self.client.send_goal(goal)
        self.client.wait_for_result()
        #get position of end effector
        position, rpy = self.get_pos()
        #get vision frames as 3D state
        #get low dimension state
        state = self.get_state(action,position)
        reward, terminal = self.get_reward(position,rpy,action)

        return state, action, reward, terminal
    
if __name__ == '__main__':
    arm = Ur5()
