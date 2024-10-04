import random

import numpy as np

import conf.configs as cf
from algorithm_factory.algorithm_SA import SA
from algorithm_factory.algorithm_heuristic_rules import Random_Choice, Least_Wait_Time_Choice, Least_Mission_Num_Choice, \
    Least_Distance_Choice, Fixed_order
from data_process.input_process import read_input

if __name__ == '__main__':
    random.seed(cf.RANDOM_SEED)
    np.random.seed(cf.RANDOM_SEED)
    inst_type_ls = ['LS-1', 'LS+1', 'IS-1', 'IS+1', 'YC-1', 'YC+1', 'QC-1', 'QC+1', 'C']
    m_num_ls = [kk for kk in range(10) for _ in range(len(inst_type_ls))]
    makespan_forall = []
    congestion_forall = []
    c_ls = []
    c_is = []
    c_yc = []
    time_forall = []

    for i in range(len(m_num_ls)):
        f = open("output_result/profile.txt", "a")  # 1000
        solu = read_input(pre='profile', inst_idx=m_num_ls[i], inst_type=inst_type_ls[int(i % 9)], mission_num=100)
        makespan, _, _ = Random_Choice(solu.init_env)
        makespan_forall.append(makespan)
    for qq in range(10):
        for i in range(len(makespan_forall)):
            if i % 9 == qq:
                print(inst_type_ls[int(i % 9)] + "\t" + str(m_num_ls[i] + 1) + "\tmakespan:\t" + str(
                    makespan_forall[i]))
