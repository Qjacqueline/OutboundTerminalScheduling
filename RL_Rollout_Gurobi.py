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
    parser.add_argument('--mission_num', type=int, default=cf.MISSION_NUM)
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
    for j in range(8):
        train_solus = []
        test_solus = []
        ls = [i for i in range(10)]
        profiles = [chr(65 + j) + '2' for _ in range(1)]
        for i in range(len(ls)):
            train_solus.append(read_input('train', str(ls[i]), profiles[i], args.mission_num))
        for i in range(len(ls)):
            test_solus.append(read_input('train', str(ls[i]), profiles[i], args.mission_num))
        for solu in train_solus:
            solu.l2a_init()
        for solu in test_solus:
            solu.l2a_init()
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
        random.seed(args.seed)
        agent = DDQN(
            eval_net=QNet(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
                          out_dim=train_solus[0].init_env.ls_num, ma_num=train_solus[0].init_env.machine_num),
            target_net=QNet(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
                            out_dim=train_solus[0].init_env.ls_num, ma_num=train_solus[0].init_env.machine_num),
            dim_action=train_solus[0].init_env.ls_num,
            device=args.device,
            gamma=args.gamma,
            epsilon=args.epsilon,
            lr=args.lr)
        # agent = DDQN(
        #     eval_net=Dueling_DQN(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
        #                   out_dim=train_solus[0].init_env.ls_num, ma_num=train_solus[0].init_env.machine_num),
        #     target_net=Dueling_DQN(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
        #                     out_dim=train_solus[0].init_env.ls_num, ma_num=train_solus[0].init_env.machine_num),
        #     dim_action=train_solus[0].init_env.ls_num,
        #     device=args.device,
        #     gamma=args.gamma,
        #     epsilon=args.epsilon,
        #     lr=args.lr)
        data_buffer = LABuffer(buffer_size=args.buffer_size)
        collector = LACollector(inst_type=args.inst_type, train_solus=train_solus, test_solus=test_solus,
                                data_buffer=data_buffer, batch_size=args.batch_size,
                                mission_num=train_solus[0].init_env.J_num_all, agent=agent,
                                rl_logger=None, save_path=args.save_path, max_num=args.max_num)
        agent.qf = torch.load(args.save_path + '/eval_' + profiles[0][0:2] + '.pkl')
        agent.qf_target = torch.load(args.save_path + '/target_' + profiles[0][0:2] + '.pkl')
        RL_flag, rollout_flag, exact_flag, exact_flag2, exact_flag3 = True, False, False, False, True
        if RL_flag:
            makespan_forall_RL, reward_forall, time_forall_RL = collector.eval(False)
        if rollout_flag:
            makespan_forall_rollout, _, time_forall_rollout = collector.rollout()
        if exact_flag:
            makespan_forall_gurobi, time_g = collector.exact(args.inst_type)
        if exact_flag3:
            makespan_forall_gurobi3, time_g3 = collector.exact3(args.inst_type)
