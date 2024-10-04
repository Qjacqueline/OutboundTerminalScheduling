#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：Port_Scheduling_New
@File    ：train_lower_agent.py
@Author  ：JacQ
@Date    ：2022/4/21 10:33
"""

import argparse
import datetime
import os
import random
import time

import numpy as np
import torch
from tensorboardX import SummaryWriter

import conf.configs as cf
from algorithm_factory.algo_utils.data_buffer import LABuffer
from algorithm_factory.algo_utils.net_models import QNet, PQNet, QLNet
from algorithm_factory.rl_algo.lower_agent import DDQN, LACollector
from data_process.input_process import read_input, read_json_from_file
from utils.log import exp_dir, Logger

logger = Logger().get_logger()


def get_args(**kwargs):
    parser = argparse.ArgumentParser()

    parser.add_argument('--inst_type', type=str, default=cf.inst_type)
    parser.add_argument('--max_num', type=int, default=5)

    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--hidden', type=int, default=64)
    parser.add_argument('--device', type=str, default='cuda:0' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--gamma', type=float, default=0.99)  # 0.9
    parser.add_argument('--epsilon', type=float, default=0.1)
    parser.add_argument('--lr', type=float, default=1e-4)
    parser.add_argument('--epoch_num', type=int, default=100000)  #
    parser.add_argument('--batch_size', type=int, default=128)
    parser.add_argument('--buffer_size', type=int, default=128000)
    parser.add_argument('-save_path', type=str, default=cf.MODEL_PATH)

    command_list = []
    for key, value in kwargs.items():
        command_list.append(f'--{key}={value}')
    return parser.parse_args(command_list)


if __name__ == '__main__':
    # ==============  Create environment  =============
    args = get_args()
    pt = args.inst_type + 'N'
    exp_dir = exp_dir(desc=f'{pt}')
    rl_logger = SummaryWriter(exp_dir)
    rl_logger.add_text(tag='parameters', text_string=str(args))
    rl_logger.add_text(tag='characteristic', text_string='original')  # 'debug'
    s_t = time.time()

    # seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    random.seed(args.seed)

    # env
    train_solus = []
    test_solus = []
    for i in range(0, 100):
        train_solus.append(read_input('train', str(i), args.inst_type))
    for i in range(0, 150):
        test_solus.append(read_input('train', str(i), args.inst_type))
    for solu in train_solus:
        solu.l2a_init()
    for solu in test_solus:
        solu.l2a_init()

    # ========================= Policy ======================
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
    #     eval_net=QLNet(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
    #                    out_dim=train_solus[0].init_env.ls_num, ma_num=train_solus[0].init_env.machine_num),
    #     target_net=QLNet(device=args.device, in_dim_max=args.max_num, hidden=args.hidden,
    #                      out_dim=train_solus[0].init_env.ls_num, ma_num=train_solus[0].init_env.machine_num),
    #     dim_action=train_solus[0].init_env.ls_num,
    #     device=args.device,
    #     gamma=args.gamma,
    #     epsilon=args.epsilon,
    #     lr=args.lr)

    # ======================== Data ==========================
    data_buffer = LABuffer(buffer_size=args.buffer_size)
    collector = LACollector(inst_type=args.inst_type, train_solus=train_solus, test_solus=test_solus,
                            data_buffer=data_buffer, batch_size=args.batch_size,
                            mission_num=train_solus[0].init_env.J_num_all, agent=agent,
                            rl_logger=rl_logger, save_path=args.save_path, max_num=args.max_num)
    # args.inst_type
    # =================== heuristic l_train ==================
    # collector.get_transition(
    #     read_json_from_file(cf.OUTPUT_SOLUTION_PATH + 'train_1_SA_10_1868.875721615327.json'), test_solus[0])
    # data_name = ['train_1_', 'train_2_', 'train_3_', 'train_4_', 'train_5_', 'train_6_', 'train_7_', 'train_8_']
    # data_name = ['train_2_']
    # collector.collect_heuristics(data_name)

    # ======================= train =======================
    for i in range(1, args.epoch_num):
        collector.collect_rl()
    e_t = time.time()
    print("training time" + str(e_t - s_t))

    os.rename(exp_dir, f'{exp_dir}_done')
    rl_logger.close()
