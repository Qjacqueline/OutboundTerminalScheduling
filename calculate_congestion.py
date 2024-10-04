# -*- coding: utf-8 -*-
# @Time    : 2023/2/18 11:01 PM
# @Author  : JacQ
# @File    : calculate_congestion.py

import random
import time

import numpy as np

import conf.configs as cf
from algorithm_factory.algo_utils.machine_cal_methods import cal_congestion
from algorithm_factory.algorithm_heuristic_rules import Least_Mission_Num_Choice
from common.iter_solution import IterSolution
from data_process.input_process import read_input
from algorithm_factory.gurobi_solver import CongestionPortModel2, solve_model, solve_model_fixed, CongestionPortModel


def cal(ls, profile):
    f = open("output_result/congestion_small_instance.txt", "a")
    env = read_input('train', str(ls), profile, mission_num=ls)  # mission_num=100,ls[i]
    # env = read_input('workload', str(ls), profile)  # mission_num=100,ls[i]
    makespan, env, _ = Least_Mission_Num_Choice(env.init_env)
    print("work")
    congestion_time = cal_congestion(env)
    solution = IterSolution(env)
    solution.iter_env = env
    model = CongestionPortModel2(solution)
    model.gamma = makespan
    model.construct(solution)
    s_t_g = time.time()
    solve_model_fixed(MLP=model.MLP, inst_idx=profile)
    e_t_g = time.time()
    congestion_time_a = model.MLP.getVars()[-1].x
    time_forall = e_t_g - s_t_g
    f.write(str(ls) + profile + "\t" + str(congestion_time) + "\t" + str(congestion_time_a) + "\t" + str(
        time_forall) + "\n")
    f.close()


if __name__ == '__main__':
    random.seed(cf.RANDOM_SEED)
    np.random.seed(cf.RANDOM_SEED)
    total_makespan = 0
    profiles = ['H2_t']  # 'H2_t','I2_t'
    ls = [5000 for _ in range(len(profiles))]
    for i in range(len(ls)):
        cal(profile=profiles[i], ls=ls[i])

    # profiles = ['low', 'medium']
    # ls = ['profile','profile']
    # for i in range(len(ls)):
    #     cal(profile=profiles[i], ls=ls[i])

    # ls = [10, 12, 27, 10, 15, 21, 10, 11, 21,
    #       10, 11, 10, 14, 15, 10, 11, 12,
    #       10, 17, 21, 10, 14, 18, 10, 16, 23]
    # profiles = ['A2_t', 'A2_t', 'A2_t', 'B2_t', 'B2_t', 'B2_t', 'C2_t', 'C2_t', 'C2_t',
    #             'D2_t', 'D2_t', 'E2_t', 'E2_t', 'E2_t', 'F2_t', 'F2_t', 'F2_t',
    #             'G2_t', 'G2_t', 'G2_t', 'H2_t', 'H2_t', 'H2_t', 'Z2_t', 'Z2_t', 'Z2_t']
    # for i in range(len(ls)):
    #     cal(profile=profiles[i], ls=ls[i])

    # ls = [1000 for i in range(8)]
    # profiles = [chr(i + 65) + '2_t' for i in range(8)]
    # ls = [1000]
    # profiles = ['Z2_t' for i in range(9)]
    # for i in range(len(ls)):
    #     cal(profile=profiles[i], ls=ls[i])

    # ls = [1000 for i in range(9)]
    # profiles = [chr(i + 65) + '2_t' for i in range(9)]
    # for i in range(len(ls)):
    #     cal(profile=profiles[i], ls=ls[i])
