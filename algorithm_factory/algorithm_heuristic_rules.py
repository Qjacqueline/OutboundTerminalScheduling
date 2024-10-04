#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：dispatching_rules.py
@Author  ：JacQ
@Date    ：2021/12/21 15:20
"""
import time
from copy import deepcopy

from algorithm_factory.algo_utils.machine_cal_methods import quay_crane_process_by_order, buffer_process_by_order, \
    exit_process_by_order, crossover_process_by_order, yard_crane_process_by_order, \
    station_process_by_random, station_process_by_least_mission_num, station_process_by_least_distance, \
    station_process_by_least_wait, output_solution, get_rnn_state_v2, station_process_by_fixed_order
from algorithm_factory.algo_utils.missions_sort_rules import sort_missions
from common.iter_solution import IterSolution
from utils.log import Logger

logger = Logger().get_logger()


def Random_Choice(port_env, buffer_flag=True):
    solution = deepcopy(port_env)
    quay_crane_process_by_order(solution)
    buffer_process_by_order(solution)
    exit_process_by_order(solution)
    station_process_by_random(solution, buffer_flag)
    crossover_process_by_order(solution, buffer_flag)
    yard_crane_process_by_order(solution, buffer_flag)
    for mission in solution.mission_list:
        mission.cal_mission_attributes(buffer_flag)
    solution.last_step_makespan = solution.cal_finish_time()
    return solution.last_step_makespan, solution, output_solution(solution)


def Fixed_order(port_env, order, buffer_flag=True):
    T1 = time.time()
    solution = deepcopy(port_env)
    quay_crane_process_by_order(solution)
    buffer_process_by_order(solution)
    exit_process_by_order(solution)
    station_process_by_fixed_order(solution, order, buffer_flag)
    crossover_process_by_order(solution, buffer_flag)
    yard_crane_process_by_order(solution, buffer_flag)
    for mission in solution.mission_list:
        mission.cal_mission_attributes(buffer_flag)
    solution.last_step_makespan = solution.cal_finish_time()
    return solution.last_step_makespan, solution, output_solution(solution)


def Random_Choice_By_Mission(solu: IterSolution, m_max_num: int, mission_num: int):
    T1 = time.time()
    state = get_rnn_state_v2(solu.iter_env, 0, m_max_num)
    for step in range(mission_num):
        if step == mission_num - 1:
            new_state = state
        else:
            new_state = get_rnn_state_v2(solu.iter_env, step + 1, m_max_num)
        state = new_state
        step += 1


def Least_Wait_Time_Choice(port_env, buffer_flag=True):
    T1 = time.time()
    solution = deepcopy(port_env)
    missions_sort_rule = 'FCFS'
    quay_crane_process_by_order(solution)
    buffer_process_by_order(solution)
    exit_process_by_order(solution)
    getattr(sort_missions, missions_sort_rule)(solution.mission_list)
    station_process_by_least_wait(solution, buffer_flag)
    crossover_process_by_order(solution, buffer_flag)
    yard_crane_process_by_order(solution, buffer_flag)
    for mission in solution.mission_list:
        mission.cal_mission_attributes(buffer_flag)
    solution.last_step_makespan = solution.cal_finish_time()
    return solution.last_step_makespan, solution, output_solution(solution)


def Least_Mission_Num_Choice(port_env, buffer_flag=True):
    T1 = time.time()
    solution = deepcopy(port_env)
    quay_crane_process_by_order(solution)
    buffer_process_by_order(solution)
    exit_process_by_order(solution)
    station_process_by_least_mission_num(solution, buffer_flag)
    crossover_process_by_order(solution, buffer_flag)
    yard_crane_process_by_order(solution, buffer_flag)
    for mission in solution.mission_list:
        mission.cal_mission_attributes(buffer_flag)
    solution.last_step_makespan = solution.cal_finish_time()
    return solution.last_step_makespan, solution, output_solution(solution)


def Least_Mission_Num_Choice_By_Mission(solu: IterSolution, m_max_num: int, mission_num: int):
    T1 = time.time()
    state = get_rnn_state_v2(solu.iter_env, 0, m_max_num)
    for step in range(mission_num):
        min_mission_num = 10000
        for station in solu.iter_env.lock_stations.values():
            if any(station.mission_list):
                cur_mission_num = len(station.mission_list)
            else:
                cur_mission_num = 0
            if cur_mission_num < min_mission_num:
                min_mission_num = cur_mission_num
        if step == mission_num - 1:
            new_state = state
        else:
            new_state = get_rnn_state_v2(solu.iter_env, step + 1, m_max_num)
        state = new_state


def Least_Distance_Choice(port_env, buffer_flag=True):
    T1 = time.time()
    solution = deepcopy(port_env)
    quay_crane_process_by_order(solution)
    buffer_process_by_order(solution)
    exit_process_by_order(solution)
    station_process_by_least_distance(solution, buffer_flag)
    crossover_process_by_order(solution, buffer_flag)
    yard_crane_process_by_order(solution, buffer_flag)
    for mission in solution.mission_list:
        mission.cal_mission_attributes(buffer_flag)
    solution.last_step_makespan = solution.cal_finish_time()
    return solution.last_step_makespan, solution, output_solution(solution)
