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
from moveit_commander import PlanningSceneInterface
from std_srvs.srv import Empty
#from ur_msgs.srv import GetRobotStateRv



JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
DURATION = 0.01
GOAL = [0.525000, 0.00, 0.854007, 0.00, 0.00, 0.00]
INIT = [0.0, -1.571, 1.571, -1.571, 0.0, 0.0]

##taking this value from Josip's Paper
max_dist = 2.004
#INIT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class Ur5():
    get_counter = 0

    def __init__(self, init_joints=INIT, goal_pose=GOAL, duration=DURATION):
        # Initialize ROS node
        rospy.init_node('ur5_env', anonymous=True)
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if index > 0:
            prefix = str(parameters)[index + len("prefix': '"):(index + len("prefix': '") + str(parameters)[index + len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name

        # Create Action Client for controlling the robot arm
        self.client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()

        # Set initial joint configuration
        self.initial = FollowJointTrajectoryGoal()
        self.initial.trajectory = JointTrajectory()
        self.initial.trajectory.joint_names = JOINT_NAMES
        self.current_joints = init_joints

        self.initial.trajectory.points = [JointTrajectoryPoint(positions=INIT, velocities=[0] * 6,
                                                               time_from_start=rospy.Duration(duration))]
        self.tf = TransformListener()
        self.goal_pose = np.array(goal_pose)
        self.base_pos = self.get_pos(link_name='base_link')
        self.duration = duration
        self.state_dim = 9
        self.action_dim = 6

        # self.initial.trajectory.point(link_name='base_link')
        # self.duration = duration
        # self.state_dim = 9
        # self.action_dim = 6

        # Initialize MoveIt PlanningSceneInterface
        self.scene = PlanningSceneInterface()
        rospy.sleep(2)  # Wait for the scene to initialize

        # Add table to the MoveIt planning scene
        self.add_table_to_scene()

        self.target_generate()

        # Initialize Gazebo reset service
        # rospy.wait_for_service('/gazebo/reset_simulation')
        # self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    # Method to add the table to the MoveIt planning scene
    def add_table_to_scene(self):
        table_pose = PoseStamped()
        table_pose.header.frame_id = "world"  # Use the correct reference frame
        table_pose.pose.position.x = 0.0  # Set to your table's position (centered at origin)
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.005  # Half of the table's height to place it on the ground
        table_pose.pose.orientation.w = 1.0

        # Add the table to the scene with correct size
        self.scene.add_box("table", table_pose, size=(1.27, 0.577, 0.03))  # Set size to match your table dimensions
        rospy.loginfo("Table added to MoveIt planning scene with correct dimensions")

    # # Method to update the table pose in the MoveIt scene
    # def update_table_pose(self):
    #     rospy.wait_for_service('/gazebo/get_model_state')
    #     try:
    #         get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    #         table_state = get_model_state("table", "world")  # "table" is the model name, "world" is the reference frame

    #         # Get the table's pose
    #         table_pose = PoseStamped()
    #         table_pose.header.frame_id = "world"
    #         table_pose.pose = table_state.pose

    #         # Update the table pose in the MoveIt scene
    #         self.scene.add_box("table", table_pose, size=(1.27, 0.577, 0.03))  # Set size as needed
    #         rospy.loginfo("Table pose updated in MoveIt scene")

    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Failed to get table pose: {e}")

    def step(self,action, object_position, timestep):
        #Execute action 

        self.client.cancel_all_goals() 

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        
        
        


        #####Finding the present joint angles
        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, timeout=1.0)
            present_joint_angles = joint_state_msg.position  # Directly assign all joint angles

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to get current joint angles: {e}. Using last known values.")


        print(action)

        action_sent = np.zeros(6)

        joint_current_angles = list(present_joint_angles)

        for i in range(len(action)):
            action_sent[i] = action[i] + joint_current_angles[i]
        
        #### action is delta_theta and present_joint_angles is theta
        # action = action + list(present_joint_angles)
        
       
       

        goal.trajectory.points = [JointTrajectoryPoint(positions=action_sent, velocities=[0]*6, 
                                                                    time_from_start=rospy.Duration(self.duration))]
        
        # np.savetxt('joint_angles.txt',goal,fmt='%d')
        self.client.send_goal(goal)
        self.client.wait_for_result()

        

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




        

        position, rpy = self.get_pos(link_name='ee_link')



        #####Finding the present joint angles
        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, timeout=1.0)
            moving_joint_angles = joint_state_msg.position  # Directly assign all joint angles

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to get current joint angles: {e}. Using last known values.")
        
        ###getting the angles after the movement
        after_moving_angles = list(moving_joint_angles)
        
        ### recent joint angles are passed in the state space
        state = self.get_state(after_moving_angles, position, object_position)

        reward, terminal, target_pos, end_eff_pos = self.get_reward(object_position,position, timestep,flag_path_tolerance_violation, flag_invalid_joints)

        return state, reward, terminal, target_pos, end_eff_pos, after_moving_angles, rpy, flag_success_reward, flag_invalid_goal,flag_invalid_joints, flag_path_tolerance_violation, flag_goal_tolerance_violation
    
    def reset(self):
        
        ###First Cancelling all the goals
        self.client.cancel_all_goals()

        self.current_joints = INIT
        self.client.send_goal(self.initial)
        self.client.wait_for_result()
        object_position = self.target_generate()
        position, rpy = self.get_pos(link_name='ee_link')
        
        state_obtained = self.get_state(INIT,position, object_position)

        return state_obtained, object_position
    
    # def reset(self):
    #     """Perform a soft reset by sending the robot to its initial joint configuration."""
    #     self.client.cancel_all_goals()
    #     self.current_joints = INIT
    #     self.client.send_goal(self.initial)
    #     self.client.wait_for_result()
    #     object_position = self.target_generate()
    #     position, rpy = self.get_pos(link_name='ee_link')
    #     state_obtained = self.get_state(INIT, position, object_position)

    #     # Check if the reset was successful
    #     if not self.is_reset_successful(state_obtained):
    #         rospy.logwarn("Soft reset failed. Performing hard reset.")
    #         return self.hard_reset()

    #     return state_obtained, object_position

    # def hard_reset(self):
    #     """Perform a hard reset using Gazebo's reset_simulation service."""
    #     try:
    #         self.reset_simulation()  # Call Gazebo's reset service
    #         rospy.loginfo("Hard reset performed using Gazebo's reset_simulation.")
    #         rospy.sleep(1)  # Wait for a moment to allow the simulation to reset
    #         self.client.send_goal(self.initial)
    #         self.client.wait_for_result()
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")

    #     # Generate a new target position
    #     object_position = self.target_generate()
    #     position, rpy = self.get_pos(link_name='ee_link')
    #     state_obtained = self.get_state(INIT, position, object_position)

    #     return state_obtained, object_position

    # def is_reset_successful(self, state):
    #     """Check if the robot has been successfully reset to the initial state."""
    #     # Here you can implement checks to determine if the robot is in the desired initial state
    #     # For simplicity, let's assume we check the joint positions
    #     joint_positions = state[:len(JOINT_NAMES)]
    #     for i, joint_position in enumerate(joint_positions):
    #         if abs(joint_position - INIT[i]) > 0.05:  # Allow some tolerance
    #             return False
    #     return True


    def get_state(self,action, position, object_position):
        #x, y, z of angular distance
        goal_pose = object_position[:3]

        #x,y, z of angualr orientation
        #goal_orient = object_position[3:]

        #print("Shape of rpy:", np.shape(rpy))
        
        #goal_dis = np.linalg.norm(goal_pose-self.base_pos)
        #print(rpy[0])
        #print(position)
        
        ####pose_6 estimates the angular distance between the goal pose and current positiuon

        dx = (goal_pose[0] - position[0])/max_dist
        dy = (goal_pose[1] - position[1])/max_dist
        dz = (goal_pose[2] - position[2])/max_dist

        #pose_6 = position - goal_pose
        #orient_6 = rpy - goal_orient
        ###pose_6 must be absplute value
        

        #dis_6 = np.linalg.norm(pose_6)
        ###absolute value

        #in_point = 1 if self.get_counter > 0 else 0
        # if len(action) == 6:
        #     action = action[:5]

        for i in range(len(action)):
            action [i] = action[i]/(2 * 3.14)
        
        state = np.concatenate((action, dx, dy, dz),axis=None)
        #state = np.concatenate((pose_6,goal_dis,action,in_point),axis=None)
        #state = state / np.linalg.norm(state)

        return state
    
   
    
    def get_reward(self,object_position,pos, timestep, flag_path_tolerance_violation, flag_invalid_joints):
        t = False

        print('Entering timestep: ', timestep)
        #Compute reward based on angular distance


        dis = np.linalg.norm(np.array([(object_position[0] - pos[0])/max_dist, (object_position[1] - pos[1])/max_dist, (object_position[2] - pos[2])/max_dist]))

        
        if ((flag_path_tolerance_violation) == 1 or (flag_invalid_joints) ==1):
            print ('Penalization for Path Tolerance')
            reward = -dis * (200 - timestep + 1) 
        else:
           if (flag_path_tolerance_violation) == 0:
             print('No Penalization applied')
             reward = -dis


             if timestep < 200:
                if abs(object_position[0] - pos[0]) <= 0.01:
                     if abs(object_position[1] - pos[1]) <= 0.01:
                         if abs(object_position[2] - pos[2]) <=0.01:
                            reward += 5
                            print ('reach distance')
                            t = True
                            print('successfully reached distance')
                            print(pos)
             else:
                if timestep == 200:
                    if abs(object_position[0] - pos[0]) <= 0.01:
                     if abs(object_position[1] - pos[1]) <= 0.01:
                         if abs(object_position[2] - pos[2]) <= 0.01:
                            reward += 10
                            print ('reach distance')
                            t = True
                            print('successfully reached distance at last timestep')
                            print(pos)
        
        

        
        
        return reward, t, object_position[:3], pos

    # def get_reward(self, object_position, pos, flag_path_tolerance_violation, flag_goal_tolerance_violation,
    #                flag_invalid_goal):
    #     """
    #     Calculate the reward for the current step.

    #     Args:
    #     - object_position: The target object's position.
    #     - pos: The current position of the end-effector.
    #     - flag_path_tolerance_violation: Flag indicating if path tolerance was violated.
    #     - flag_goal_tolerance_violation: Flag indicating if goal tolerance was violated.
    #     - flag_invalid_goal: Flag indicating if an invalid goal was set.

    #     Returns:
    #     - reward: Calculated reward based on the current state.
    #     - t: Termination flag indicating if the episode should terminate.
    #     - target_pos: Target position for reference.
    #     - end_eff_pos: End-effector position for reference.
    #     """
    #     t = False  # Initialize termination flag
    #     dis = np.linalg.norm(
    #         object_position[:3] - pos)  # Calculate the distance between end-effector and target position

    #     # Collision detection and reward calculation
    #     if flag_path_tolerance_violation == 1 or flag_goal_tolerance_violation == 1:
    #         reward = -10  # Apply a negative reward as a penalty if a collision is detected
    #         rospy.logwarn("Collision detected, applying negative reward.")
    #     else:
    #         # If no collision, calculate reward based on distance
    #         reward = -dis  # Closer distance results in a higher reward (closer to target)

    #     # Additional reward logic for reaching the target position
    #     if self.get_counter < 200:
    #         if abs(object_position[0] - pos[0]) < 0.1 and abs(object_position[1] - pos[1]) < 0.1 and abs(
    #                 object_position[2] - pos[2]) < 0.1:
    #             if flag_goal_tolerance_violation == 0:
    #                 reward += 10  # Additional reward for successfully reaching the target
    #                 rospy.loginfo('Successfully reached the target position.')
    #                 t = True  # Update termination flag
    #     else:
    #         reward += 0  # No additional reward if beyond maximum steps

    #     self.get_counter += 1  # Increment step counter

    #     return reward, t, object_position[:3], pos

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
    
    # def desk_vis(self,goal):
    #     rospy.wait_for_service("gazebo/delete_model")
    #     rospy.wait_for_service("gazebo/spawn_sdf_model")
    #     delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        
    #     s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        
    #     orient = Quaternion(*tf.transformations.quaternion_from_euler(1.571, 0, 0))
    #     origin_pose = Pose(Point(goal[0],goal[1],goal[2]), orient)
        

    #     with open('/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/ur5_single_arm_tufts/models/ur_desk/model.sdf',"r") as f:
    #         reel_xml = f.read()
        
    #     for row in [1]:
    #         for col in range(1):
    #             reel_name = "reel_%d_%d" % (row,col)
    #             delete_model(reel_name)
    #             pose = deepcopy(origin_pose)
    #             pose.position.x = origin_pose.position.x + 0

    #             #if pose.position.x > 3:
    #                 #pose.position.x = 0.506

    #             pose.position.y = origin_pose.position.y + 0
                    
    #             pose.position.z = origin_pose.position.z + 0
    #             s(reel_name, reel_xml, "", pose, "world")
    #             #print("spawnobj")

    # def desk_generate(self):
    #     # rand_x, rand_y, rand_z= np.random.uniform(-0.04,0.04), np.random.uniform(-0.3,0.1), np.random.uniform(-0.6,0)

    #     # # get_target_pose
    #     # self.goal_pose = np.array(GOAL)
    #     # self.goal_pose[0] += 0
    #     # self.goal_pose[1] += 0
    #     # self.goal_pose[2] += 0
    #     central_position = [0, 0, 0]
    #     self.desk_vis(central_position)
    







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

    # If you need to periodically update the table position
    # rospy.Timer(rospy.Duration(1), lambda event: arm.update_table_pose())