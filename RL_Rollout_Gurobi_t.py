# -*- coding: utf-8 -*-
# @Time    : 2022/9/26 8:48 AM
# @Author  : JacQ
# @File    : RL_Rollout_Gurobi.py
# -*- coding: utf-8 -*-

import argparse
import datetime
import os
import random
import time
from copy import deepcopy

import numpy as np
import torch
from tensorboardX import SummaryWriter

import conf.configs as cf
from algorithm_factory.algo_utils.data_buffer import LABuffer
from algorithm_factory.algo_utils.machine_cal_methods import get_state_n
from algorithm_factory.algo_utils.net_models import QNet, PQNet, Dueling_DQN
from algorithm_factory.rl_algo.lower_agent import DDQN, LACollector
from common import PortEnv
from common.iter_solution import IterSolution
from data_process.input_process import read_input, read_json_from_file, write_env_to_file
from utils.log import exp_dir, Logger

logger = Logger().get_logger()


def get_args(**kwargs):
    parser = argparse.ArgumentParser()
    parser.add_argument('--inst_type', type=str, default=cf.inst_type)
    parser.add_argument('--max_num', type=int, default=5)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--hidden', type=int, default=64)
    parser.add_argument('--device', type=str, default='cuda:0' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--gamma', type=float, default=0.9)  # 0.9
    parser.add_argument('--epsilon', type=float, default=0.5)
    parser.add_argument('--lr', type=float, default=1e-4)
    parser.add_argument('--epoch_num', type=int, default=60)
    parser.add_argument('--batch_size', type=int, default=256)
    parser.add_argument('--buffer_size', type=int, default=128000)
    parser.add_argument('-save_path', type=str, default=cf.MODEL_PATH)
    command_list = []
    for key, value in kwargs.items():
        command_list.append(f'--{key}={value}')
    return parser.parse_args(command_list)


if __name__ == '__main__':
    args = get_args()
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    random.seed(args.seed)
    '''large'''
    # ls = [100 for _ in range(50)]
    # profiles = ['A2_t' for _ in range(50)]
    ls = [600 for k in range(0, 10)]  # , 500, 1000
    profiles = [chr(65 + k) + '2_t' for k in range(0, 6)]
    '''small'''
    # ls = [10, 13, 27, 10, 15, 21, 10, 11, 21,
    #       10, 11, 10, 14, 14, 10, 11, 12,
    #       10, 17, 21, 10, 14, 18, 10, 16, 23]
    # profiles = ['A2_t', 'A2_t', 'A2_t', 'B2_t', 'B2_t', 'B2_t', 'C2_t', 'C2_t', 'C2_t',
    #             'D2_t', 'D2_t', 'E2_t', 'E2_t', 'E2_t', 'F2_t', 'F2_t', 'F2_t',
    #             'G2_t', 'G2_t', 'G2_t', 'H2_t', 'H2_t', 'H2_t', 'Z2_t', 'Z2_t', 'Z2_t']
    # ls = [1000]
    # profiles = ['Z2_t']
    '''mode choice'''
    RL_flag, rollout_flag, exact_flag, exact_flag2, exact_flag3 = True, True, False, False, False
    for i in range(len(ls)):
        train_solu = read_input('train', str(ls[i]), profiles[i], mission_num=ls[i])
        test_solu = read_input('train', str(ls[i]), profiles[i], mission_num=ls[i])
        train_solu.l2a_init()
        test_solu.l2a_init()
        train_solus = [train_solu]
        test_solus = [test_solu]
        agent = DDQN(
            eval_net=QNet(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
                          out_dim=train_solu.init_env.ls_num, ma_num=train_solu.init_env.machine_num),
            target_net=QNet(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
                            out_dim=train_solu.init_env.ls_num, ma_num=train_solu.init_env.machine_num),
            dim_action=train_solu.init_env.ls_num,
            device=args.device,
            gamma=args.gamma,
            epsilon=args.epsilon,
            lr=args.lr)
        # agent = DDQN(
        #     eval_net=Dueling_DQN(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
        #                   out_dim=train_solu.init_env.ls_num, ma_num=train_solu.init_env.machine_num),
        #     target_net=Dueling_DQN(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
        #                     out_dim=train_solu.init_env.ls_num, ma_num=train_solu.init_env.machine_num),
        #     dim_action=train_solu.init_env.ls_num,
        #     device=args.device,
        #     gamma=args.gamma,
        #     epsilon=args.epsilon,
        #     lr=args.lr)
        data_buffer = LABuffer(buffer_size=args.buffer_size)
        collector = LACollector(inst_type=args.inst_type, train_solus=train_solus, test_solus=test_solus,
                                data_buffer=data_buffer, batch_size=args.batch_size,
                                mission_num=train_solu.init_env.J_num_all, agent=agent,
                                rl_logger=None, save_path=args.save_path, max_num=args.max_num)
        agent.qf = torch.load(args.save_path + '/eval_' + profiles[i][0:2] + '.pkl')  # profiles[i][:2]
        agent.qf_target = torch.load(args.save_path + '/target_' + profiles[i][0:2] + '.pkl')
        f = open("output_result/RLtt.txt", "a")
        f.write("L\t" + profiles[i] + " " + str(ls[i]) + "\t")
        if RL_flag:
            makespan, reward, time = collector.eval(False)
            f.write("RL " + "\t" + str(makespan[0]) + "\t" + str(time[0]) + "\t")
            f.close()
        if rollout_flag:
            makespan, solu, time = collector.rollout2()
            f = open("output_result/RLtt.txt", "a")
            f.write("rollout    " + "\t" + str(makespan[0]) + "\t" + str(time[0]) + "\t")
            f.close()
        if exact_flag:
            makespan, time = collector.exact(profiles[i])
            f = open("output_result/RL.txt", "a")
            f.write("exact" + "\t" + str(makespan[0]) + "\t" + str(time[0]) + "\t")
            f.close()