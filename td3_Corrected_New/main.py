from Shengyuan_xie_envnew import Ur5
#from env2 import Ur5_vision
#from DDPG import DDPG
from TD3 import TD3
#from TD3_vision import TD3_vision
import numpy as np 
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import torch
import time
import argparse
import os
import numpy as np
import rospy
import actionlib
import time
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
import math


# Define the base directory where you want to save log files
base_log_dir = "/home/robocupathome/arkaur5_ws/src/contexualaffordance/Ur5_DRL"

# Make sure the directory exists, if not, create it
if not os.path.exists(base_log_dir):
    os.makedirs(base_log_dir)



# File paths for logging rewards
train_reward_filename = os.path.join(base_log_dir, 'Septemp_training_rewards_whole.txt')
test_reward_filename = os.path.join(base_log_dir, 'Septemp_testing_rewards_whole.txt')
joint_angles_filename = os.path.join(base_log_dir, '23rd_Septemp_Joint_Angles_whole.txt')
rpy_filename = os.path.join(base_log_dir, '23rd_Septempemp_RPY_whole.txt')

max_ts = 200
##time duration in seconds
DELTA = 0.02

##maximum allowed latency
MAX_LATENCY = 1
MAX_NOISE = 0.1

        






def get_time(start_time):
    m, s = divmod(int(time.time()-start_time), 60)
    h, m = divmod(m, 60)

    print ('Total time spent: %d:%02d:%02d' % (h, m, s))

def img_transform(img_memory, mode, frame_size=4):
    assert type(img_memory).__module__ == np.__name__, 'data type is not numpy'
    assert mode == 'img2txt' or mode == 'txt2img', 'Please use correct mode name 1.img2txt 2.txt2img'

    if mode == 'img2txt':
        size = img_memory.shape[0]
        img_memory = img_memory.reshape((size,-1))
    
    if mode == 'txt2img':
        size = img_memory.shape[0]
        h = w = np.sqrt(img_memory.shape[1] / (2 * frame_size))
        img_memory = img_memory.reshape((size,2,frame_size,h,w))
    
    return img_memory

def train(args, env, model):

    ###setting the saving path for the trained model
    if not os.path.exists(args.path_to_model+args.model_name+args.model_date):
        os.makedirs(args.path_to_model+args.model_name+args.model_date)

    #training reward list or cumulative reward list storing cumulative rewards during
    ### training
    
    #testing reward and steps list, which stores the rewards obtained and no. of steps
    ##during the evaluation process
    #test_reward_list, test_step_list = [], []
    #total_actor_loss_list, total_critic_loss_list = [], []
    start_time = time.time()
    if args.pre_train:
        #load pre_trained model 
        try:
            model.load_model(args.path_to_model+args.model_name, args.model_date_+'/')
            print ('load model successfully')
        except:
            print ('fail to load model, check the path of models')

        #print ('start random exploration for adding experience')

        ####Loading the TD3 Model
        #if args.model_name == 'TD3':
            #state = env.reset()

        #for step in range(args.random_exploration):
            ######Chnaged for Baxter's 7 joint angles
            #if args.model_name == 'TD3':
                #state_, action, reward, terminal = env.uniform_exploration(np.random.uniform(-1,1,7)*args.action_bound*7)
                #model.store_transition(state,action,reward,state_,terminal)
                #state = state_
                #if terminal:
                    #state = env.reset()
        #total_reward_list = np.loadtxt(args.path_to_model+args.model_name+args.model_date_+'/reward.txt')
        #test_reward_list = np.loadtxt(args.path_to_model+args.model_name+args.model_date_+'/test_reward.txt')
        #test_step_list = np.loadtxt(args.path_to_model+args.model_name+args.model_date_+'/test_step.txt')

    print ('start training')
    model.mode(mode='train')

    #training for observation with TD3 Algorithm

    train_episodes_list = []
    total_reward_list = []
    dis_to_target_list = []
    smoothness_action_cost = []
    joint_angles_episode = []
    rpy_episode = []
    #Median_test_reward = []
    for epoch in range(0, 1000):

        ###creating an empty list for storing training episodes
        ### and cumulative reward in each episode
        

        if args.model_name == 'TD3':
            state, object_position = env.reset()

            # ##Creating History of Transitioning States
            # state_history = [None for _ in range(max_ts)]
            # random_latencies = np.random.uniform(0, MAX_LATENCY, (max_ts, 1))
            # random_noises = np.random.uniform(-MAX_NOISE, MAX_NOISE, (max_ts, 9))
            # previous_latency = 0
            
            ###Creating a History of Storing Actions in an Episode
            action_history = [None for _ in range(max_ts)]

        total_reward = 0
        distance_to_target = 0
        joint_angles_timestep = []
        rpy_timestep = []
        for i in range(args.train_step):
            # state_history[i] = state
            # if args.model_name == 'TD3':
            #     ####latency is added in current state
            #     if i > 0 and i < 200:
            #         latency = random_latencies[i][0] #self.np_random.uniform(0, MAX_LATENCY)
            #         latency = min(i * DELTA, latency)
            #         # latency can't be greater than the last latency and the delta time since then
            #         latency = min(previous_latency + DELTA, latency)
            #         # store the latest latency for the next step
            #         previous_latency = latency

            #         if latency == 0:
            #            lat_state = state
            #         else:
            #             # the latency is a linear interpolation between the two discrete states that correspond
            #             ratio = round(latency/DELTA, 6)
            #             earlier_state = state_history[i-math.ceil(ratio)]
            #             earlier_state_portion = (ratio-math.floor(ratio))
            #             later_state = state_history[i-math.floor(ratio)]
            #             try:
            #                lat_state = [x *  earlier_state_portion + y * (1 - earlier_state_portion) for x, y in zip(earlier_state, later_state)]
            #                state = lat_state
            #             except:
            #                print("Error happened")
            #     ####Noise Randomization is added to the State Space
            #     if i < 200:
            #         state = np.array(state) + random_noises[i]
                action = model.choose_action(state)
                action_history[i] = action
                state_, reward, terminal, target_pos, end_eff_pos, present_joint_angles, rpy, flag_success_reward, flag_invalid_goal,flag_invalid_joints, flag_path_tolerance_violation, flag_goal_tolerance_violation = env.step(action, object_position, i)
                print(terminal)
                model.store_transition(state,action,reward,state_,terminal)
                state = state_
                total_reward = total_reward + (0.99)**i * reward

                joint_angles_timestep.append(present_joint_angles)
                rpy_timestep.append(rpy)

                if i > 100 and i < 200:
                    dis_diff = (np.linalg.norm(end_eff_pos - target_pos))**2
                    distance_to_target += dis_diff

                if model.memory_counter > args.random_exploration:
                    model.Learn()
                
                if ((flag_path_tolerance_violation) == 1 or (flag_invalid_joints == 1)):
                    print("Resetting to Path Tolerance")
                    state, object_position = env.reset()
                    
                if (terminal==True):
                    print("Breaking out from the episode")
                    deltas = [None for _ in range(i-1)]
                    for i in range(0, i-1):
                        deltas[i] = np.mean(np.power(action_history[i] - action_history[i+1], 2) / 6.28)
                    deltas = np.array(deltas)

                    final_continuity_cost = 100 * np.mean(deltas)
                    smoothness_action_cost.append(final_continuity_cost)         
                    break
        total_reward_list.append(total_reward)
        train_episodes_list.append(epoch)
        final_distance_to_target = 0.01 * distance_to_target
        dis_to_target_list.append(final_distance_to_target)
        joint_angles_episode.append(joint_angles_timestep)
        rpy_episode.append(rpy_timestep)

        if (terminal == False):
            ###Evaluation of Continuity Cost of an Episode
            deltas = [None for _ in range(max_ts-1)]
            for i in range(0, max_ts-1):
                deltas[i] = np.mean(np.power(action_history[i] - action_history[i+1], 2) / (2 * 3.14))
            deltas = np.array(deltas)

            final_continuity_cost = 100 * np.mean(deltas)
            smoothness_action_cost.append(final_continuity_cost)

        
        print ('epoch:', epoch,  '||',  'Reward:', total_reward, '||', 'Dis_to_Target:', final_distance_to_target, '||', 'Continuity_Cost:', final_continuity_cost)
        with open(train_reward_filename, "a") as file:
            file.write(f"epoch: {epoch} || Train_reward: {total_reward} || Final_Dis_Target:{final_distance_to_target} || Continuity_Cost:{final_continuity_cost} \n")

        with open(joint_angles_filename, "a") as file:
            file.write(f"joint_angles: {joint_angles_episode} \n")

        with open(rpy_filename, "a") as file:
            file.write(f"rpy: {rpy_episode} \n")


        #begin testing and record the evalation metrics
        if (epoch+1) % args.test_epoch == 0:
            model.save_model(args.path_to_model+args.model_name, args.model_date+'/')

            ####Saving the Actor Critic Loss Plots for Training
            #model.plot_loss(args.path_to_model+args.model_name, args.model_date+'/', epoch)

            # test_reward, epochs_step = test(args, env, model)
            #print(test_reward)
            #print(epochs_step)
            #median_reward = np.median(test_reward)
            #Median_test_reward.append(median_reward)
            
            #model.plot_loss(args.path_to_model+args.model_name, args.model_date+'/', epoch)
            
            model.mode(mode='train')
            # print ('finish testing')

            # np.append(test_reward_list,avg_reward)
            #test_reward_list.append(avg_reward)

            # np.append(test_step_list,avg_step)
            #test_step_list.append(avg_step)

            #plt.figure()
            #plt.plot(epochs_step, test_reward, color='r')
            #plt.ylabel('test_reward')
            #plt.xlabel('testing epoch')
            #plt.title(f'test_reward_trained(Epoch {epoch + 1})')
            #plt.savefig(args.path_to_model+args.model_name+args.model_date+f'/test_reward_trained_{epoch+1}.png')
            #plt.close()
            #np.savetxt(args.path_to_model+args.model_name+args.model_date+f'/test_reward_{epoch+1}.txt',np.array(test_reward_list))

            #plt.figure()
            #plt.plot(np.arange(len(test_step_list)), test_step_list)
            #plt.ylabel('test_step')
            #plt.xlabel('training epoch / testing epoch')
            #plt.title(f'test_step_(Epoch {epoch + 1})')
            #plt.savefig(args.path_to_model+args.model_name+args.model_date+f'/test_step_epoch_{epoch+1}.png')
            #plt.close()
            #np.savetxt(args.path_to_model+args.model_name+args.model_date+f'/test_step_{epoch+1}.txt',np.array(test_step_list))

            #plt.figure()
            #plt.plot(train_episodes_list[-10:], total_reward_list[-10:], color='g')
            #plt.ylabel('Total_reward')
            #plt.xlabel('training epoch')
            #plt.title(f'Training_reward_(Epoch {epoch + 1})')
            #plt.savefig(args.path_to_model+args.model_name+args.model_date+f'/training_reward_{epoch+1}.png')
            #plt.close()
            #np.savetxt(args.path_to_model+args.model_name+args.model_date+f'/training_reward_{epoch+1}.txt',np.array(total_reward_list))
            #get_time(start_time)
    
    ####Plotting for Training Rewards
    plt.figure()
    plt.plot(train_episodes_list, total_reward_list, color='g')
    plt.ylabel('Total_reward')
    plt.xlabel('training epoch')
    plt.title('Training_reward_total')
    plt.savefig(args.path_to_model+args.model_name+args.model_date+f'/training_reward_{epoch+1}.png')
    plt.close()
    
    ####Plotting for Testing Rewards
    #plt.figure()
    #plt.plot(train_episodes_list, Median_test_reward, color='Red')
    #plt.ylabel('Median_reward')
    #p#lt.xlabel('training epoch')
    #plt.title('Testing_reward_median_epoch')
    #plt.savefig(args.path_to_model+args.model_name+args.model_date+f'/Median_testing_reward_{epoch+1}.png')
    #plt.close()



    ####Plotting for Distance To Target
    plt.figure()
    plt.plot(train_episodes_list, dis_to_target_list, color='violet')
    plt.ylabel('Distance to the Target')
    plt.xlabel('training epoch')
    plt.title('Training_Reaching')
    plt.savefig(args.path_to_model+args.model_name+args.model_date+f'/training_Dis_to_Target{epoch+1}.png')
    plt.close()

    ####Plotting for Continuity of Smoothness
    plt.figure()
    plt.plot(train_episodes_list, smoothness_action_cost, color='brown')
    plt.ylabel('Continuity of Action Smoothness')
    plt.xlabel('training epoch')
    plt.title('Training_Smoothness')
    plt.savefig(args.path_to_model+args.model_name+args.model_date+f'/training_Smoothness{epoch+1}.png')
    plt.close()




            
    

def test(args,env, model):
    model.mode(mode='test')
    print ('start to test the model')
    try:
        model.load_model(args.path_to_model+args.model_name, args.model_date_+'/')
        #model.load_model('/home/robocupathome/arkaur5_ws/src/contexualaffordance/RL_trialTD3/06_06_2024')
        print(args.path_to_model+args.model_name, args.model_date_+'/')
        print ('load model successfully')
    except Exception as e:
        rospy.logwarn(e)
        print ('fail to load model, check the path of models')

    total_reward_list = []
    epochs_list = []
    dis_to_target_list = []
    smoothness_test_cost = []
    #testing for vision observation
    for epoch in range(args.test_epoch):
        if args.model_name == 'TD3':
            state, object_pos = env.reset()
            # state_history = [None for _ in range(max_ts)]
            # random_noises_test = np.random.uniform(-MAX_NOISE, MAX_NOISE, (max_ts, 9))
            # random_latencies_test = np.random.uniform(0, MAX_LATENCY, (max_ts, 1))
            # previous_latency = 0
            action_history = [None for _ in range(max_ts + 1)]
        total_reward = 0
        distance_to_target = 0
        for step in range(args.test_step):
            # state_history[step] = state
            # if args.model_name == 'TD3':
            #     ####latency is added in current state
            #     if step > 0 and step < 200:
            #         latency = random_latencies_test[step][0] #self.np_random.uniform(0, MAX_LATENCY)
            #         latency = min(step * DELTA, latency)
            #         # latency can't be greater than the last latency and the delta time since then
            #         latency = min(previous_latency + DELTA, latency)
            #         # store the latest latency for the next step
            #         previous_latenpresent_joint_anglescy = latency

            #         if latency == 0:
            #            lat_state = state
            #         else:
            #             # the latency is a linear interpolation between the two discrete states that correspond
            #             ratio = round(latency/DELTA, 6)
            #             earlier_state = state_history[step-math.ceil(ratio)]
            #             earlier_state_portion = (ratio-math.floor(ratio))
            #             later_state = state_history[step-math.floor(ratio)]
            #             try:
            #                lat_state = [x *  earlier_state_portion + y * (1 - earlier_state_portion) for x, y in zip(earlier_state, later_state)]
            #                state = lat_state
            #             except:
            #                print("Error happened")
            #     ####Noise Randomization is added to the State Space
            #     if step < 200:
            #         state = np.array(state) + random_noises_test[step]
                action = model.choose_action(state)
                action_history[step] = action
                state_, reward, terminal, target_pos, end_eff_pos, present_joint_angles, rpy, flag_success_reward, flag_invalid_goal,flag_invalid_joints, flag_path_tolerance_violation, flag_goal_tolerance_violation = env.step(action, object_pos, step)
                state = state_
                total_reward = total_reward + (0.99)**step * reward

                if step > 100 and step < 200:
                    dis_diff = (np.linalg.norm(end_eff_pos - target_pos))**2
                    distance_to_target += dis_diff

                if ((flag_path_tolerance_violation) == 1 or (flag_invalid_joints == 1)):
                    print('Resetting for Path Tolerance')
                    state, object_position = env.reset()

                if (terminal==True):
                    print("Breaking out from the episode")

                    deltas = [None for _ in range(step-1)]
                    for i in range(0, step-1):
                        deltas[i] = np.mean(np.power(action_history[i] - action_history[i+1], 2) / 6.28)
                    deltas = np.array(deltas)

                    final_continuity_cost = 100 * np.mean(deltas)
                    smoothness_test_cost.append(final_continuity_cost)
                                
                    break
        total_reward_list.append(total_reward)
        final_distance_to_target = 0.01 * distance_to_target
        dis_to_target_list.append(final_distance_to_target)

        
  
        ###Evaluation of Continuity Cost of an Episode

        if (terminal == False):
            deltas = [None for _ in range(max_ts-1)]
            for i in range(0, max_ts-1):
                deltas[i] = np.mean(np.power(action_history[i] - action_history[i+1], 2) / 6.28)
            deltas = np.array(deltas)
            final_continuity_cost = 100 * np.mean(deltas)
            smoothness_test_cost.append(final_continuity_cost)

        print ('testing_epoch:', epoch,  '||',  'Reward:', total_reward, '||', 'Distance_to_target:', final_distance_to_target, '||', 'Continuity_Cost:', final_continuity_cost)
        ###Writing in file
        with open(test_reward_filename, "a") as file:
            file.write(f"epoch: {epoch} || Test_Reward: {total_reward} || Final_Distance_to_Target: {final_distance_to_target} || Continuity_Cost: {final_continuity_cost} \n")

        ####adding each iterations
        epochs_list.append(epoch)


    #average_reward = np.mean(np.array(total_reward_list))
    #average_step = 0 if steps_list == [] else np.mean(np.array(steps_list))

    return total_reward_list, epochs_list


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    #select env to be used
    parser.add_argument('--env_name', default='empty')
    #select model to be used
    parser.add_argument('--model_name', default='TD3')
    #Folder name saved as date
    parser.add_argument('--model_date', default='/23_09_2024')
    #Folder stored with trained model weights, which are used for transfer learning
    parser.add_argument('--model_date_', default='/23_09_2024')
    parser.add_argument('--pre_train', default=False)

    #####has to be changed
    parser.add_argument('--path_to_model', default='/home/robocupathome/rosur5_ws/src/contexualaffordance/DRLMODELTD3')
    #The maximum action limit

    ####has to be chnaged
    # parser.add_argument('--action_bound', default=np.pi/72, type=float) #pi/36 for reachings
    parser.add_argument('--train_epoch', default=1000, type=int)
    parser.add_argument('--train_step', default=200, type=int)
    parser.add_argument('--test_epoch', default=10, type=int)
    parser.add_argument('--test_step', default=200, type=int)
    #exploration (randome action generation) steps before updating the model
    parser.add_argument('--random_exploration', default=1000, type=int)
    #store the model weights and plots after epoch number
    parser.add_argument('--epoch_store', default=10, type=int)
    #Wether to use GPU
    parser.add_argument('--cuda', default=True)
    parser.add_argument('--mode', default='train')
    args = parser.parse_args()

    #assert args.env_name == 'empty' or 'vision', 'env name: 1.empty 2.vision'
    assert args.env_name == 'empty', 'env name: 1.empty'
    if args.env_name == 'empty': env = Ur5()
    #if args.env_name == 'vision': env = Ur5_vision()

    #assert args.model_name == 'TD3_vision' or 'TD3' or 'DDPG', 'model name: 1.TD3_vision 2.TD3 3.DDPG'
    #assert args.model_name == 'DDPG', 'model name: 1.DDPG '
    #if args.model_name == 'TD3_vision': model = TD3_vision(a_dim=env.action_dim,s_dim=env.state_dim,cuda=args.cuda)
    if args.model_name == 'TD3': model = TD3(a_dim=env.action_dim,s_dim=env.state_dim,cuda=args.cuda)
    #if args.model_name == 'DDPG': model = DDPG(a_dim=env.action_dim,s_dim=env.state_dim,cuda=args.cuda)

    assert args.mode == 'test' or 'test', 'mode: 1.train 2.test'
    if args.mode == 'train': 
        train(args, env, model)

    if args.mode == 'test': 
        env.duration = 0.1
        test(args, env, model)
