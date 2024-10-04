# -*- coding: utf-8 -*-
# @Time    : 2023/2/13 9:48 PM
# @Author  : JacQ
# @File    : calculate_LB.py
from copy import deepcopy

from algorithm_factory.algo_utils.machine_cal_methods import cal_LB1, cal_LB2, cal_LB3
from algorithm_factory.algorithm_heuristic_rules import Least_Mission_Num_Choice
from data_process.input_process import read_input

if __name__ == '__main__':
    for j in range(1):
        m_num_ls = [600, 1350, 4000]
        inst_type_ls = ['low', 'medium', 'heavy']
        pre = "workload"
        lb1s, lb2s, lb3s, ms = [], [], [], []
        for i in range(len(m_num_ls)):
            solu = read_input(pre, str(m_num_ls[i]), inst_type_ls[i], m_num_ls[i])
            _, lb_env, _ = Least_Mission_Num_Choice(deepcopy(solu.init_env))
            lb1 = cal_LB1(lb_env)
            lb2, r_lb2 = cal_LB2(lb_env)
            lb3 = cal_LB3(lb_env, r_lb2)
            solu.l2a_init()
            lb1s.append(lb1)
            lb2s.append(lb2)
            lb3s.append(lb3)
        for i in range(len(lb1s)):
            print(max(lb1s[i], lb2s[i], lb3s[i]), ",")
