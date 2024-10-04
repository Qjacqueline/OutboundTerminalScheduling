#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：port_scheduling
@File    ：algorithm_interface.py
@Author  ：JacQ
@Date    ：2021/12/22 11:01
"""
from algorithm_factory.algorithm_SA import SA
from algorithm_factory.algorithm_exact import Exact_Method
from algorithm_factory.algorithm_heuristic_rules import Random_Choice, Least_Wait_Time_Choice, Least_Mission_Num_Choice, \
    Least_Distance_Choice
from utils.log import Logger

logger = Logger(name='root').get_logger()


def algorithm_interface(test_env, algorithm_index, buffer_flag=True):

    if algorithm_index == 0:
        return Exact_Method(test_env)
    elif algorithm_index == 1:
        return Random_Choice(test_env, buffer_flag)
    elif algorithm_index == 2:
        return Least_Wait_Time_Choice(test_env, buffer_flag)
    elif algorithm_index == 3:
        return Least_Mission_Num_Choice(test_env, buffer_flag)
    elif algorithm_index == 4:
        return Least_Distance_Choice(test_env, buffer_flag)
    elif algorithm_index == 5:
        return SA(test_env).run()
    else:
        logger.error("wrong input.")
        exit()
