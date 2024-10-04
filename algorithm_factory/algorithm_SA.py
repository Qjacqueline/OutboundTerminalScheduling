#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：Port_Scheduling
@File    ：SA.py
@Author  ：JacQ
@Date    ：2022/2/15 14:26
"""
import random
import time
from copy import deepcopy

import matplotlib.pyplot as plt
import numpy as np

import conf.configs as Cf
from algorithm_factory.algo_utils.machine_cal_methods import output_solution
from common.iter_solution import IterSolution
from data_process.input_process import write_json_to_file
from utils.log import Logger

logger = Logger().get_logger()
random.seed(Cf.RANDOM_SEED)
np.random.seed(Cf.RANDOM_SEED)


def eval_solution(best_solution, tmp_solution, temp):
    makespan1 = best_solution.last_step_makespan
    makespan2 = tmp_solution.last_step_makespan
    dc = makespan2 - makespan1
    p = max(1e-1, np.exp(-dc / temp))
    if makespan2 < makespan1:
        return tmp_solution
    elif np.random.rand() <= p:
        return tmp_solution
    else:
        return best_solution


class SA(object):
    def __init__(self, iter_solu: IterSolution, T0: float = Cf.T0, Tend: float = Cf.TEND, rate: float = Cf.RATE,
                 buffer_flag: bool = True):
        self.T0: float = T0
        self.Tend: float = Tend
        self.rate: float = rate
        self.iter_solu: IterSolution = iter_solu
        self.buffer_flag: bool = buffer_flag
        self.iter_x: list = list()
        self.iter_y: list = list()
        self.best_makespan: float = float('inf')

    def run(self, stop_time=float('inf')):
        T1 = time.time()
        count = 0
        tf1 = True
        tf2 = True
        tf3 = True
        best_solution: IterSolution = deepcopy(self.iter_solu)
        best_makespan: float = self.iter_solu.last_step_makespan
        record = []
        while tf1 or tf2 or tf3:
            count += 1
            tmp_solution, flag = self.get_new_solution(deepcopy(self.iter_solu))
            if flag == 'end':
                break
            if tmp_solution.last_step_makespan < best_makespan:
                best_makespan = tmp_solution.last_step_makespan
                best_solution = tmp_solution
            self.iter_solu = eval_solution(best_solution, tmp_solution, self.T0)
            self.T0 *= self.rate
            self.iter_x.append(count)
            self.iter_y.append(self.iter_solu.last_step_makespan)
            if time.time() - T1 > stop_time and tf1:
                record.append(best_makespan)
                print("best_makespan: " + str(best_makespan), end=" ")
                tf1 = False
            # if time.time() - T1 > 60 and tf1:
            #     record.append(best_makespan)
            #     print("best_makespan: " + str(best_makespan), end=" ")
            #     tf1 = False
            # if time.time() - T1 > 500 and tf2:
            #     record.append(best_makespan)
            #     print("best_makespan: " + str(best_makespan), end=" ")
            #     tf2 = False
            # if time.time() - T1 > 14000 and tf3:
            #     record.append(best_makespan)
            #     print("best_makespan: " + str(best_makespan), end=" ")
            #     print()
            #     tf3 = False
        self.iter_solu.reset()
        return record

    @staticmethod
    def get_new_solution(solution: IterSolution):
        action = random.randint(0, 5)
        reward, flag = solution.step_v1(action)
        return solution, flag

    def plot_result(self):
        plt.rcParams["font.sans-serif"] = ["SimHei"]
        plt.rcParams["font.family"] = "sans-serif"
        plt.rcParams['axes.unicode_minus'] = False
        iterations = self.iter_x
        best_record = self.iter_y
        plt.plot(iterations, best_record)
        plt.title('iteration')
        plt.show()
