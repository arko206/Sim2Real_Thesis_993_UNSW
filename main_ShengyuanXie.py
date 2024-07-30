from env_ShengyuanXie import Ur5
from DDPG_ShengyuanXie import DDPG
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import torch
import time
import argparse
import os


def get_time(start_time):
    m, s = divmod(int(time.time() - start_time), 60)
    h, m = divmod(m, 60)
    print('Total time spent: %d:%02d:%02d' % (h, m, s))


def train(args, env, model):
    if not os.path.exists(args.path_to_model + args.model_name + args.model_date):
        os.makedirs(args.path_to_model + args.model_name + args.model_date)

    total_reward_list = []
    test_reward_list, test_step_list = [], []
    start_time = time.time()
    if args.pre_train:
        try:
            model.load_model(args.path_to_model + args.model_name, args.model_date_ + '/')
            print('load model successfully')
        except:
            print('fail to load model, check the path of models')

        print('start random exploration for adding experience')
        state = env.reset()
        for step in range(args.random_exploration):
            action = np.random.uniform(-1, 1, env.action_dim) * args.action_bound
            state_, reward, terminal = env.step(action)
            model.store_transition(state, action, reward, state_, terminal)
            state = state_ if not terminal else env.reset()

        total_reward_list = np.loadtxt(args.path_to_model + args.model_name + args.model_date_ + '/reward.txt')
        test_reward_list = np.loadtxt(args.path_to_model + args.model_name + args.model_date_ + '/test_reward.txt')
        test_step_list = np.loadtxt(args.path_to_model + args.model_name + args.model_date_ + '/test_step.txt')

    print('start training')
    model.mode(mode='train')

    for epoch in range(args.train_epoch):
        state = env.reset()
        total_reward = 0
        for i in range(args.train_step):
            action = model.choose_action(state)
            state_, reward, terminal = env.step(action * args.action_bound)
            model.store_transition(state, action, reward, state_, terminal)
            state = state_ if not terminal else env.reset()
            total_reward += reward
            if model.memory_counter > args.random_exploration:
                model.Learn()

        total_reward_list.append(total_reward)
        print('epoch:', epoch, '||', 'Reward:', total_reward)

        if (epoch + 1) % args.test_epoch == 0:
            model.save_model(args.path_to_model + args.model_name, args.model_date + '/')
            avg_reward, avg_step = test(args, env, model)
            model.mode(mode='train')
            print('finish testing')
            test_reward_list.append(avg_reward)
            test_step_list.append(avg_step)

            plt.figure()
            plt.plot(np.arange(len(test_reward_list)), test_reward_list)
            plt.ylabel('test_reward')
            plt.xlabel('training epoch / testing epoch')
            plt.savefig(args.path_to_model + args.model_name + args.model_date + '/test_reward.png')
            plt.close()
            np.savetxt(args.path_to_model + args.model_name + args.model_date + '/test_reward.txt',
                       np.array(test_reward_list))

            plt.figure()
            plt.plot(np.arange(len(test_step_list)), test_step_list)
            plt.ylabel('test_step')
            plt.xlabel('training epoch / testing epoch')
            plt.savefig(args.path_to_model + args.model_name + args.model_date + '/test_step.png')
            plt.close()
            np.savetxt(args.path_to_model + args.model_name + args.model_date + '/test_step.txt',
                       np.array(test_step_list))

            plt.figure()
            plt.plot(np.arange(len(total_reward_list)), total_reward_list)
            plt.ylabel('Total_reward')
            plt.xlabel('training epoch')
            plt.savefig(args.path_to_model + args.model_name + args.model_date + '/reward.png')
            plt.close()
            np.savetxt(args.path_to_model + args.model_name + args.model_date + '/reward.txt',
                       np.array(total_reward_list))

            get_time(start_time)


def test(args, env, model):
    model.mode(mode='test')
    print('start to test the model')
    try:
        model.load_model(args.path_to_model + args.model_name, args.model_date_ + '/')
        print(args.path_to_model + args.model_name, args.model_date_ + '/')
        print('load model successfully')
    except:
        print('fail to load model, check the path of models')

    total_reward_list = []
    steps_list = []
    for epoch in range(args.test_epoch):
        state = env.reset()
        total_reward = 0
        for step in range(args.test_step):
            action = model.choose_action(state, noise=None)
            state_, reward, terminal = env.step(action * args.action_bound)
            state = state_ if not terminal else env.reset()
            total_reward += reward
        total_reward_list.append(total_reward)
        print('testing_epoch:', epoch, '||', 'Reward:', total_reward)

    average_reward = np.mean(np.array(total_reward_list))
    average_step = 0 if steps_list == [] else np.mean(np.array(steps_list))

    return average_reward, average_step


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--env_name', default='empty')
    parser.add_argument('--model_name', default='DDPG')
    parser.add_argument('--model_date', default='/21_07_2024')
    parser.add_argument('--model_date_', default='/21_07_2024')
    parser.add_argument('--pre_train', default=False)
    parser.add_argument('--path_to_model', default='/home/zhitao/RL_UR5/')
    parser.add_argument('--action_bound', default=np.pi / 72, type=float)
    parser.add_argument('--train_epoch', default=500, type=int)
    parser.add_argument('--train_step', default=200, type[int)
    parser.add_argument('--test_epoch', default=10, type[int)
    parser.add_argument('--test_step', default=200, type[int)
    parser.add_argument('--random_exploration', default=1000, type[int)
    parser.add_argument('--epoch_store', default=10, type[int)
    parser.add_argument('--cuda', default=False)
    parser.add_argument('--mode', default='train')
    args = parser.parse_args()

    env = Ur5()
    model = DDPG(a_dim=env.action_dim, s_dim=8, cuda=args.cuda)

    if args.mode == 'train':
        train(args, env, model)

    if args.mode == 'test':
        env.duration = 0.1
    test(args, env, model)
