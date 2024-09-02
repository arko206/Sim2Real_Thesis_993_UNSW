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
from math import sin, cos
from sensor_msgs.msg import JointState
#from ur_msgs.srv import GetRobotStateRv


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
DURATION = 0.01

####Goal is the position of the cube in Gazebo,
####x,y,z position of cube; r,p,y oreintation of cube
GOAL = [0.525000,0.00,0.854007,0.00,0.00,0.00]

### INIT refers to the initial joint angles of the arm
### 6 joints are given initial angles
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
        self.action_dim = 6

        #self.desk_generate()
        self.target_generate()

    
    
    
    ### Step function in responsible for moving the arm
    ### according to the predicted joint angles from the actor network
    def step(self,action, object_position):
        #Execute action 

        self.client.cancel_all_goals()

        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        #set action joint limit
        #action consists of the predicted joint angles

        #### Current joint values has been added to predicted angles, to
        #### have higher range of movements
        self.current_joints += action
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
        

        goal.trajectory.points = [JointTrajectoryPoint(positions=action_sent, velocities=[0]*6, 
                                                                    time_from_start=rospy.Duration(self.duration))]
        
        # np.savetxt('joint_angles.txt',goal,fmt='%d')

        ###client server moves the arm according to joint angle
        #### values form action_sent
        self.client.send_goal(goal)
        self.client.wait_for_result()

        ###Check wrong joint trajectories, path tolerance from the server
        flag_success_reward = 0
        flag_invalid_goal = 0
        flag_invalid_joints = 0
        flag_path_tolerance_violation = 0
        flag_goal_tolerance_violation = 0

        result = self.client.get_result()
        
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            error_message = result.error_string
            if result.error_code == FollowJointTrajectoryResult.INVALID_GOAL:
                rospy.logwarn(f"Invalid goal: {error_message}")
                flag_invalid_goal = 1

            elif result.error_code == FollowJointTrajectoryResult.INVALID_JOINTS:
                rospy.logwarn(f"Invalid joints: {error_message}")
                flag_invalid_joints = 1

            elif result.error_code == FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
                rospy.logwarn(f"Path tolerance violated: {error_message}")
                flag_path_tolerance_violation = 1

            elif result.error_code == FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                rospy.logwarn(f"Goal tolerance violated: {error_message}")
                flag_goal_tolerance_violation = 1

            else:
                rospy.logwarn(f"Unknown error code {result.error_code}: {error_message}")
        else:
            rospy.loginfo("Trajectory successfully executed.")
            flag_success_reward = 1




        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, timeout=1.0)
            present_joint_angles = joint_state_msg.position  # Directly assign all joint angles
        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to get current joint angles: {e}. Using last known values.")

        position, rpy = self.get_pos(link_name='ee_link')
      
        #### invalid joints flag and goal tolerance has been added to state argument, in order to mark these trajectories
        #### to be invalid and violation of goal positions to ensure proper learning
      
        state = self.get_state(action, position, object_position, flag_invalid_joints, flag_goal_tolerance_violation)

        reward, terminal, target_pos, end_eff_pos = self.get_reward(object_position,position, flag_path_tolerance_violation, flag_goal_tolerance_violation, flag_invalid_goal)

        return state, reward, terminal, target_pos, end_eff_pos, present_joint_angles, rpy, flag_success_reward, flag_invalid_goal,flag_invalid_joints, flag_path_tolerance_violation, flag_goal_tolerance_violation


    def reset(self):
        
        ###First Cancelling all the goals
        self.client.cancel_all_goals()

        self.current_joints = INIT
        self.client.send_goal(self.initial)
        self.client.wait_for_result()
        object_position = self.target_generate()
        position, rpy = self.get_pos(link_name='ee_link')
        flag_invalid_joints = 0
        flag_goal_tolerance_violation = 0

        #print(position)
        #print("*******")
        #print(rpy)

        state_obtained = self.get_state(INIT,position, object_position, flag_invalid_joints, flag_goal_tolerance_violation)

        return state_obtained, object_position

    def get_state(self,action, position, object_position, flag_invalid_joints, flag_goal_tolerance_violation):
        #x, y, z of angular distance
        goal_pose = object_position[:3]

        #x,y, z of angualr orientation
        #goal_orient = object_position[3:]

        #print("Shape of rpy:", np.shape(rpy))
        
        #goal_dis = np.linalg.norm(goal_pose-self.base_pos)
        #print(rpy[0])
        #print(position)
        
        ####pose_6 estimates the angular distance between the goal pose and current positiuon
        pose_6 = position - goal_pose
        #orient_6 = rpy - goal_orient
        ###pose_6 must be absplute value
        

        #dis_6 = np.linalg.norm(pose_6)
        ###absolute value

        #in_point = 1 if self.get_counter > 0 else 0
        # if len(action) == 6:
        #     action = action[:5]
        
        state = np.concatenate((action, pose_6, flag_invalid_joints, flag_goal_tolerance_violation, ),axis=None)
        #state = np.concatenate((pose_6,goal_dis,action,in_point),axis=None)
        #state = state / np.linalg.norm(state)

        return state
    
    def get_reward(self,object_position,pos, flag_path_tolerance_violation, flag_goal_tolerance_violation, flag_invalid_goal):
        t = False
        #Compute reward based on angular distance
        dis = np.linalg.norm(object_position[:3]-pos)
        #add regularization term

        #compute orientation distance
        #dis_a = np.linalg.norm(object_position[3:]-rpy)
        # print(self.goal_pose[3])
        # print(rpy[0])
        #r_a = -0.5 * dis_a

        if (flag_path_tolerance_violation) == 1:
            reward = -dis - 5
        else:
           reward = - dis 

        collision_coeff = 0

        if (flag_invalid_goal) == 1:
            reward -= 20

        ####If Collisiion occurs
        if dis == 0.1:
            collision_coeff = 1
            reward  = reward - collision_coeff

        if self.get_counter < 200:
            if abs(object_position[0] - pos[0]) < 0.1:
                if abs(object_position[1] - pos[1]) < 0.1:
                    if abs(object_position[2] - pos[2]) < 0.1:
                        if (flag_goal_tolerance_violation == 0):
                            reward = reward * (200 -self.get_counter) + 1
                            print ('reach distance')
                            t = True
                            print('successfully reached distance')
                            print(pos)
        else:
            if self.get_counter == 200:
                if abs(object_position[0] - pos[0]) < 0.1:
                    if abs(object_position[1] - pos[1]) < 0.1:
                        if abs(object_position[2] - pos[2]) < 0.1:
                            if (flag_goal_tolerance_violation == 0):
                                reward = reward * (200 -self.get_counter) + 10
                                print ('reach distance')
                                t = True
                                print('successfully reached distance at last timestep')
                                print(pos)
            else:
                reward += 0

        self.get_counter += 1
        
        return reward, t, object_position[:3], pos
    
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
        

        with open('/home/robocupathome/arkaur5_ws/src/contexualaffordance/models/box_red/model.sdf',"r") as f:
            reel_xml = f.read()
        
        for row in [1]:
            for col in range(1):
                reel_name = "reel_%d_%d" % (row,col)
                delete_model(reel_name)
                pose = deepcopy(origin_pose)
                pose.position.x = origin_pose.position.x + 0

                #if pose.position.x > 3:
                    #pose.position.x = 0.506

                pose.position.y = origin_pose.position.y + 0
                    
                pose.position.z = origin_pose.position.z + 0
                s(reel_name, reel_xml, "", pose, "world")
                #print("spawnobj")

    def target_generate(self):
        rand_x, rand_y, rand_z= np.random.uniform(-0.04,0.04), np.random.uniform(-0.3,0.1), np.random.uniform(-0.6,0)

        # get_target_pose
        self.goal_pose = np.array(GOAL)
        self.goal_pose[0] += 0
        self.goal_pose[1] += 0
        self.goal_pose[2] += 0
        self.target_vis(self.goal_pose)
        return self.goal_pose
    
    def desk_vis(self,goal):
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        
        s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        
        orient = Quaternion(*tf.transformations.quaternion_from_euler(1.571, 0, 0))
        origin_pose = Pose(Point(goal[0],goal[1],goal[2]), orient)
        

        with open('/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/ur5_single_arm_tufts/models/ur_desk/model.sdf',"r") as f:
            reel_xml = f.read()
        
        for row in [1]:
            for col in range(1):
                reel_name = "reel_%d_%d" % (row,col)
                delete_model(reel_name)
                pose = deepcopy(origin_pose)
                pose.position.x = origin_pose.position.x + 0

                #if pose.position.x > 3:
                    #pose.position.x = 0.506

                pose.position.y = origin_pose.position.y + 0
                    
                pose.position.z = origin_pose.position.z + 0
                s(reel_name, reel_xml, "", pose, "world")
                #print("spawnobj")

    def desk_generate(self):
        # rand_x, rand_y, rand_z= np.random.uniform(-0.04,0.04), np.random.uniform(-0.3,0.1), np.random.uniform(-0.6,0)

        # # get_target_pose
        # self.goal_pose = np.array(GOAL)
        # self.goal_pose[0] += 0
        # self.goal_pose[1] += 0
        # self.goal_pose[2] += 0
        central_position = [0, 0, 0]
        self.desk_vis(central_position)
    







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
