from New_Reward_env_ShengyuanXie import Ur5
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
base_log_dir = "/home/arka/newark_ws/src/Ur5_TD3"

# Make sure the directory exists, if not, create it
if not os.path.exists(base_log_dir):
    os.makedirs(base_log_dir)



# File paths for logging rewards
test_reward_filename = os.path.join(base_log_dir, 'New_08August_rewards_whole_Real_TestOnly.txt')
actrion_history_filename = os.path.join(base_log_dir, 'New_ActionHistory_whole_Real_TestOnly.txt')

max_ts = 50
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
    #start_time = time.time()
    if args.pre_train:
        #load pre_trained model 
        try:
            base_path = "/home/arka/newark_ws/src/Ur5_TD3/DRLMODELTD3TD3/2_08_2024/"
            actor_path = os.path.join(base_path, "actor.pth")
            critic_path = os.path.join(base_path, "critic.pth")
            optim_a_path = os.path.join(base_path, "optim_a.pth")
            optim_c_path = os.path.join(base_path, "optim_c.pth")
            model.load_model(actor_path, critic_path, optim_a_path, optim_c_path)
            print('Model loaded successfully')
        except:
            print ('fail to load model, check the path of models')

    print ('start testing')
    #model.mode(mode='test')

    ####Saving the Actor Critic Loss Plots for Training
    #model.plot_loss(args.path_to_model+args.model_name, args.model_date+'/', epoch)

    test_reward, epochs_step = test(args, env, model)
    print ('finish testing')


def test(args,env, model):
    model.mode(mode='test')
    print ('start to test the model')
    try:
        #model.load_model(args.path_to_model+args.model_name, args.model_date_+'/')
        #model.load_model('/home/robocupathome/arkaur5_ws/src/contexualaffordance/RL_trialTD3/06_06_2024')
        #print(args.path_to_model+args.model_name, args.model_date_+'/')
        base_path = "/home/arka/newark_ws/src/Ur5_TD3/DRLMODELTD3TD3/2_08_2024/"
        actor_path = os.path.join(base_path, "actor.pth")
        critic_path = os.path.join(base_path, "critic.pth")
        optim_a_path = os.path.join(base_path, "optim_a.pth")
        optim_c_path = os.path.join(base_path, "optim_c.pth")
        model.load_model(actor_path, critic_path, optim_a_path, optim_c_path)
        print('Model loaded successfully')
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
            state = env.reset()
            state_history = [None for _ in range(max_ts)]
            random_noises_test = np.random.uniform(-MAX_NOISE, MAX_NOISE, (max_ts, 11))
            random_latencies_test = np.random.uniform(0, MAX_LATENCY, (max_ts, 1))
            previous_latency = 0
            action_history = [None for _ in range(max_ts + 1)]
        total_reward = 0
        distance_to_target = 0
        for step in range(args.test_step):
            state_history[step] = state
            if args.model_name == 'TD3':
                ####latency is added in current state
                if step > 0 and step < max_ts:
                    latency = random_latencies_test[step][0] #self.np_random.uniform(0, MAX_LATENCY)
                    latency = min(step * DELTA, latency)
                    # latency can't be greater than the last latency and the delta time since then
                    latency = min(previous_latency + DELTA, latency)
                    # store the latest latency for the next step
                    previous_latency = latency

                    if latency == 0:
                       lat_state = state
                    else:
                        # the latency is a linear interpolation between the two discrete states that correspond
                        ratio = round(latency/DELTA, 6)
                        earlier_state = state_history[step-math.ceil(ratio)]
                        earlier_state_portion = (ratio-math.floor(ratio))
                        later_state = state_history[step-math.floor(ratio)]
                        try:
                           lat_state = [x *  earlier_state_portion + y * (1 - earlier_state_portion) for x, y in zip(earlier_state, later_state)]
                           state = lat_state
                        except:
                           print("Error happened")
                ####Noise Randomization is added to the State Space
                if step < max_ts:
                    state = np.array(state) + random_noises_test[step]
                action = model.choose_action(state,noise=False)
                action_history[step] = action
                state_, reward, terminal, target_pos, end_eff_pos = env.step(action * args.action_bound)
                state = state_
                total_reward = total_reward + (0.99)**step * reward

                if step > (max_ts//2) and step < max_ts:
                    dis_diff = (np.linalg.norm(end_eff_pos - target_pos))**2
                    distance_to_target += dis_diff

                if terminal:
                    print("Arm can reach the target")
                    env.reset()
        total_reward_list.append(total_reward)
        final_distance_to_target = 0.01 * distance_to_target
        dis_to_target_list.append(final_distance_to_target)

        ###Evaluation of Continuity Cost of an Episode
        deltas = [None for _ in range(max_ts-1)]
        for i in range(0, max_ts-1):
            deltas[i] = np.mean(np.power(action_history[i] - action_history[i+1], 2) / 4)
        deltas = np.array(deltas)

        final_continuity_cost = 100 * np.mean(deltas)
        smoothness_test_cost.append(final_continuity_cost)

        print ('testing_epoch:', epoch,  '||',  'Reward:', total_reward, '||', 'Distance_to_target:', final_distance_to_target, '||', 'Continuity_Cost:', final_continuity_cost)
        ###Writing in file
        with open(test_reward_filename, "a") as file:
            file.write(f"epoch: {epoch} || Test_Reward: {total_reward} || Final_Distance_to_Target: {final_distance_to_target} || Continuity_Cost: {final_continuity_cost} \n")
        
        with open(actrion_history_filename, "a") as file:
            file.write(f"epoch: {epoch} || Action_History: {action_history} \n")

        ####adding each iterations
        epochs_list.append(epoch)


    #average_reward = np.mean(np.array(total_reward_list))
    #average_step = 0 if steps_list == [] else np.mean(np.array(steps_list))

    return total_reward_list, epochs_list


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--env_name', default='empty')
    parser.add_argument('--model_name', default='TD3')
    parser.add_argument('--model_date', default='/2_08_2024')
    parser.add_argument('--model_date_', default='/2_08_2024')
    parser.add_argument('--pre_train', default=True)
    parser.add_argument('--path_to_model', default='/home/arka/newark_ws/src/Ur5_TD3/DRLMODELTD3')
    parser.add_argument('--action_bound', default=np.pi/72, type=float)
    parser.add_argument('--train_epoch', default=30, type=int)
    parser.add_argument('--train_step', default=10, type=int)
    parser.add_argument('--test_epoch', default=50, type=int)
    parser.add_argument('--test_step', default=50, type=int)
    parser.add_argument('--random_exploration', default=1000, type=int)
    parser.add_argument('--epoch_store', default=10, type=int)
    parser.add_argument('--cuda', default=False)
    parser.add_argument('--mode', default='train')
    args = parser.parse_args()

    env = Ur5()
    model = TD3(a_dim=env.action_dim, s_dim=11, cuda=args.cuda)

    if args.mode == 'train':
        train(args, env, model)

    if args.mode == 'test':
        env.duration = 0.1
        test(args, env, model)

    print("End")