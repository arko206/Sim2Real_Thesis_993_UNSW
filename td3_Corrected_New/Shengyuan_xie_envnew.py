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
import time
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
        self.state_dim = 7
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


    def step(self,action, object_position, timestep):
        #####Finding the present joint angles
        try:
            joint_state_msg = rospy.wait_for_message('/joint_states', JointState, timeout=1.0)

            ##getting the present joint angles
            present_joint_angles = joint_state_msg.position  # Directly assign all joint angles

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to get current joint angles: {e}. Using last known values.")

        print('Prediction from ACtor')
        print(action)

        action_sent = np.zeros(6)

        joint_current_angles = list(present_joint_angles)
        

        print('Angle from Previous State')
        print(joint_current_angles)

        ##joint angles are added
        for i in range(len(action)):
            if i == 2:
                action_sent[i] == action[i] * 0.5 * np.pi + joint_current_angles[i]
            else:
                action_sent[i] = action[i] *  np.pi + joint_current_angles[i]
            

        # Adjust joints degree which has bound [-pi,pi] and apply constraints
        for i in range(6):
            if i == 2:
                if action_sent[i] > 0.5 * np.pi:
                    action_sent[i] = 0.5 * np.pi
                elif action_sent[i] < -0.5 * np.pi:
                    action_sent[i] = -0.5 * np.pi
                else:
                    action_sent[i] = action_sent[i]
            else:
                if action_sent[i] > np.pi:
                    action_sent[i] = np.pi
                elif action_sent[i] < -np.pi:
                    action_sent[i] = -np.pi
                else:
                    action_sent[i] = action_sent[i]

        print('Joint angle for New State')
        print(action_sent)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        goal.path_tolerance = [JointTolerance(name=joint, position=0.3) for joint in JOINT_NAMES]

        #### given some angular values and velocity
        goal.trajectory.points = [JointTrajectoryPoint(positions=action_sent,
                                                                    time_from_start=rospy.Duration(2 * self.duration))]
        
        
    

        self.client.send_goal(goal)
        # print(time.time())
        self.client.wait_for_result()

        # Check goal status
        goal_state = self.client.get_state()

        if goal_state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal successfully reached.")
        elif goal_state == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("Goal aborted.")
        elif goal_state == actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn("Goal preempted.")
        else:
            rospy.loginfo(f"Goal state: {goal_state}")


        
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

        print('After Movement')
        print(after_moving_angles)
        
        ### recent joint angles are passed in the state space
        state = self.get_state(after_moving_angles, position, object_position)

        reward, terminal, target_pos, end_eff_pos = self.get_reward(object_position,position, timestep,flag_path_tolerance_violation, flag_invalid_joints)

        return state, reward, terminal, target_pos, end_eff_pos, after_moving_angles, rpy, flag_success_reward, flag_invalid_goal,flag_invalid_joints, flag_path_tolerance_violation, flag_goal_tolerance_violation
    
    def reset(self):
        
        ###First Cancelling all the goals
        #self.client.cancel_all_goals()

        # Set initial joint configuration
        self.initial = FollowJointTrajectoryGoal()
        self.initial.trajectory = JointTrajectory()
        self.initial.trajectory.joint_names = JOINT_NAMES
        INIT = [0.0, -1.571, 1.571, -1.571, 0.0, 0.0]
        

        self.initial.trajectory.points = [JointTrajectoryPoint(positions=INIT, velocities=[0] * 6,
                                                               time_from_start=rospy.Duration(DURATION))]

        # self.current_joints = INIT
        self.client.send_goal(self.initial)
        self.client.wait_for_result()
        object_position = self.target_generate()
        position, rpy = self.get_pos(link_name='ee_link')
        
        state_obtained = self.get_state(INIT,position, object_position)

        return state_obtained, object_position
    
    

    def get_state(self,angular_values, position, object_position):
        #x, y, z of angular distance
        goal_pose = object_position[:3]

        

        # dx = (goal_pose[0] - position[0])/max_dist
        # dy = (goal_pose[1] - position[1])/max_dist
        # dz = (goal_pose[2] - position[2])/max_dist

        position_diff = (np.linalg.norm(goal_pose - position))/max_dist



        for i in range(len(angular_values)):
            if i == 2:
              angular_values[i] = angular_values[i]/(0.5 * np.pi)
            else:
                angular_values[i] = angular_values[i]/(np.pi)


        
        
        state = np.concatenate((angular_values, position_diff),axis=None)
        #state = np.concatenate((pose_6,goal_dis,action,in_point),axis=None)
        #state = state / np.linalg.norm(state)
        print("State Representation is: ")
        print(state)

        return state
    
   
    def get_reward(self,object_position,pos, timestep, flag_path_tolerance_violation, flag_invalid_joints):
        t = False

        print('Entering timestep: ', timestep)
        #Compute reward based on angular distance
        


        # dis = np.linalg.norm(np.array([(object_position[0] - pos[0])/max_dist, (object_position[1] - pos[1])/max_dist, (object_position[2] - pos[2])/max_dist]))
        dis = np.linalg.norm(object_position[:3] - pos)
        print("The distance is :", dis)

        reward = -dis
        
        if ((flag_path_tolerance_violation) == 1 or (flag_invalid_joints) ==1):
            print ('Penalization for Path Tolerance')
            reward -= 10
        else:
        #    if (flag_path_tolerance_violation) == 0:
        #      print('No Penalization applied')

            if  dis <= 0.1:
                reward += 10
                print ('reach distance')
                t = True
                print('successfully reached distance at timestep', timestep)
                print(pos)

            # #checkpoint: ddistance. only; 
            # #timestep: t<T , reward should be higher
            #  if timestep < 200:
            #     if abs(object_position[0] - pos[0]) <= 0.1:
            #          if abs(object_position[1] - pos[1]) <= 0.1:
            #              if abs(object_position[2] - pos[2]) <=0.1:
            #                 reward += 5
           
            #                 print ('reach distance')
            #                 t = True
            #                 print('successfully reached distance')
            #                 print(pos)
            #  else:
            #     if timestep == 200:
            #         if abs(object_position[0] - pos[0]) <= 0.1:
            #          if abs(object_position[1] - pos[1]) <= 0.1:
            #              if abs(object_position[2] - pos[2]) <= 0.1:
            #                 reward += 10
            #                 print ('reach distance')
            #                 t = True
            #                 print('successfully reached distance at last timestep')
            #                 print(pos)
        
        

        
        
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
    


if __name__ == '__main__':
    arm = Ur5()

    # If you need to periodically update the table position
    # rospy.Timer(rospy.Duration(1), lambda event: arm.update_table_pose())