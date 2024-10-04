# -*- coding: utf-8 -*-
# @Time    : 2023/1/3 4:26 PM
# @Author  : JacQ
# @File    : gurobi.py

import random
import time

import numpy as np
import torch
from gurobipy import GRB
import conf.configs as cf
from data_process.input_process import read_input
from algorithm_factory.gurobi_solver import CongestionPortModel, solve_model
from utils.log import Logger

logger = Logger().get_logger()

if __name__ == '__main__':
    seed = 0
    np.random.seed(seed)
    torch.manual_seed(seed)
    random.seed(seed)

    # env
    train_solus = []
    test_solus = []

    ''' single obj '''
    # m_num_ls = [15]
    # inst_type_ls = ['B2_t']
    # gammas = [1316.52679188132]
    # # thetas = [1500]

    ''' multi obj epsilon'''
    # delta = 1
    # m_num_ls = [16 for _ in range(delta)]
    # inst_type_ls = ['Z2_t' for _ in range(delta)]
    # gammas = [876.271496288637, 898.034116453539]

    ''' profile '''
    inst_type_ls = ['LS-1', 'LS+1', 'IS-1', 'IS+1', 'YC-1', 'YC+1', 'QC-1', 'QC+1', 'C']
    m_num_ls = [kk for kk in range(1, 10) for _ in range(len(inst_type_ls))]
    makespan_forall = []
    congestion_forall = []
    c_ls = []
    c_is = []
    c_yc = []
    time_forall = []
    for i in range(len(m_num_ls)):
        f = open("output_result/profile.txt", "a")
        solu = read_input(pre='profile', inst_idx=m_num_ls[i], inst_type=inst_type_ls[int(i % 9)], mission_num=10)
        solu.l2a_init()
        model = CongestionPortModel(solu)
        # model.gamma = gamma[i]
        model.construct()
        s_t_g = time.time()
        solve_model(MLP=model.MLP, inst_idx=inst_type_ls[int(i % 9)] + '_' + str(m_num_ls[i]), solved_env=solu,
                    tag='_exact',
                    X_flag=False, Y_flag=False)
        e_t_g = time.time()
        if model.MLP.status != GRB.Status.OPTIMAL:
            makespan_forall.append('inf')
            congestion_forall.append('inf')
            c_yc.append('inf')
            c_is.append('inf')
            c_ls.append('inf')
        else:
            makespan_forall.append(model.MLP.getVars()[-5].x)
            congestion_forall.append(model.MLP.getVars()[-4].x)
            c_ls.append(model.MLP.getVars()[-3].x)
            c_is.append(model.MLP.getVars()[-2].x)
            c_yc.append(model.MLP.getVars()[-1].x)
        time_forall.append(e_t_g - s_t_g)
