#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：Port_Scheduling_New
@File    ：operators.py
@Author  ：JacQ
@Date    ：2022/4/22 9:31
"""
from algorithm_factory.algo_utils import machine_cal_methods, mission_cal_methods
from algorithm_factory.algo_utils.missions_sort_rules import sort_missions
from common.port_env import PortEnv

from utils.log import Logger

logger = Logger().get_logger()


def inter_relocate_random_station_random_mission_to_random_machine(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    while True:
        s_a = machine_cal_methods.find_random_machine(iter_env.lock_stations)
        if s_a.mission_list:
            break
    s_b = machine_cal_methods.find_random_machine(iter_env.lock_stations, s_a)
    if s_a == s_b:
        return 'not end'
    m_a = mission_cal_methods.find_random_one_mission_one_machine(s_a)
    pos = mission_cal_methods.find_least_wait_delta_position_for_mission(m_a, s_b)
    logger.debug("relocate_operator3:" + s_a.idx + "-" + m_a.idx + '->' + s_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    missions_sort_rule = 'FCFS'
    machine_cal_methods.process_relocate_operator(m_a, pos, s_a, s_b, buffer_flag)
    getattr(sort_missions, missions_sort_rule)(iter_env.mission_list)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    getattr(sort_missions, missions_sort_rule)(iter_env.mission_list)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


def inter_relocate_random_machine_longest_mission_to_random_machine(iter_env: PortEnv, buffer_flag: bool = True):
    s_a = machine_cal_methods.find_random_machine(iter_env.lock_stations)
    if not s_a.mission_list:
        return 'end'
    s_b = machine_cal_methods.find_random_machine(iter_env.lock_stations, s_a)
    if s_a == s_b:
        return 'not end'
    m_a = mission_cal_methods.find_longest_mission_for_machine(s_a)
    pos = mission_cal_methods.find_least_wait_delta_position_for_mission(m_a, s_b)
    logger.debug("relocate_operator2:" + s_a.idx + "-" + m_a.idx + '->' + s_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    missions_sort_rule = 'FCFS'
    machine_cal_methods.process_relocate_operator(m_a, pos, s_a, s_b, buffer_flag)
    getattr(sort_missions, missions_sort_rule)(iter_env.mission_list)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    getattr(sort_missions, missions_sort_rule)(iter_env.mission_list)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


def inter_relocate_latest_station_random_mission_to_random_machine(iter_env: PortEnv, buffer_flag: bool = True):
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    s_b = machine_cal_methods.find_random_machine(iter_env.lock_stations, s_a)
    if s_a == s_b:
        return 'not end'
    m_a = mission_cal_methods.find_random_one_mission_one_machine(s_a)
    pos = mission_cal_methods.find_least_wait_delta_position_for_mission(m_a, s_b)
    logger.debug("station inter relocate:" + s_a.idx + "-" + m_a.idx + '->' + s_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    machine_cal_methods.process_relocate_operator(m_a, pos, s_a, s_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


def inter_relocate_latest_station_random_mission_to_earliest_station(iter_env: PortEnv, buffer_flag: bool = True):
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    s_b = machine_cal_methods.find_earliest_machine(iter_env.lock_stations)
    if s_a == s_b:
        return 'not end'
    m_a = mission_cal_methods.find_random_one_mission_one_machine(s_a)
    pos = mission_cal_methods.find_least_wait_delta_position_for_mission(m_a, s_b)
    logger.debug("relocate_operator4:" + s_a.idx + "-" + m_a.idx + '->' + s_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    machine_cal_methods.process_relocate_operator(m_a, pos, s_a, s_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


def inter_relocate_latest_station_longest_mission_to_earliest_machine(iter_env: PortEnv, buffer_flag: bool = True):
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    s_b = machine_cal_methods.find_earliest_machine(iter_env.lock_stations)
    if s_a == s_b:
        return 'not end'
    m_a = mission_cal_methods.find_longest_mission_for_machine(s_a)
    pos = mission_cal_methods.find_least_wait_delta_position_for_mission(m_a, s_b)
    logger.debug("relocate_operator1:" + s_a.idx + "-" + m_a.idx + '->' + s_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    machine_cal_methods.process_relocate_operator(m_a, pos, s_a, s_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


def inter_relocate_latest_yard_crane_random_mission(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    y_a = machine_cal_methods.find_latest_machine(iter_env.yard_cranes)
    m_a = mission_cal_methods.find_random_one_mission_one_machine(y_a)
    s_a = iter_env.lock_stations[m_a.machine_list[4]]
    s_b = machine_cal_methods.find_random_machine(iter_env.lock_stations, s_a)
    if s_a == s_b:
        return 'not end'
    pos = mission_cal_methods.find_least_wait_delta_position_for_mission(m_a, s_b)
    logger.debug("yard inter relocate:" + "station: " + s_a.idx + " " + m_a.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    machine_cal_methods.process_relocate_operator(m_a, pos, s_a, s_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


# check
def inner_swap_latest_station_longest_mission(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    m_a, m_b = mission_cal_methods.find_longest_adjacent_two_mission_one_machine(s_a)
    logger.debug("inner swap:" + "station: " + s_a.idx + " " + m_a.idx + "<->" + m_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.process_inner_swap(s_a, m_a, m_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


def inner_swap_latest_yard_crane_random_mission(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    y_a = machine_cal_methods.find_latest_machine(iter_env.yard_cranes)
    m_a = mission_cal_methods.find_random_one_mission_one_machine(y_a)
    s_a = iter_env.lock_stations(m_a.machine_list[4])
    m_b = mission_cal_methods.find_adjacent_mission_for_mission(m_a, s_a)
    logger.debug("yard inner swap:" + "station: " + s_a.idx + " " + m_a.idx + "<->" + m_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.process_inner_swap(s_a, m_a, m_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


# check
def inner_relocate_latest_station_random_mission(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    m_a = mission_cal_methods.find_random_one_mission_one_machine(s_a)
    pos = mission_cal_methods.find_position_for_machine_to_relocate_inner_machine(m_a, s_a)
    logger.debug("station inner relocate:" + "station: " + s_a.idx + " " + m_a.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.process_inner_relocate(s_a, m_a, pos, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


# check
def inner_relocate_latest_station_longest_mission(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    m_a = mission_cal_methods.find_longest_mission_for_machine(s_a)
    pos = mission_cal_methods.find_position_for_machine_to_relocate_inner_machine(m_a, s_a)
    logger.debug("inner relocate:" + "station: " + s_a.idx + " " + m_a.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.process_inner_relocate(s_a, m_a, pos, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


# check
def inner_relocate_latest_yard_crane_random_mission(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    y_a = machine_cal_methods.find_latest_machine(iter_env.yard_cranes)
    m_a = mission_cal_methods.find_random_one_mission_one_machine(y_a)
    s_a = iter_env.lock_stations[m_a.machine_list[4]]
    pos = mission_cal_methods.find_position_for_machine_to_relocate_inner_machine(m_a, s_a)
    logger.debug("yard inner relocate:" + "station: " + s_a.idx + " " + m_a.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.process_inner_relocate(s_a, m_a, pos, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


# check
def inter_swap_latest_station_longest_mission_earliest_machine(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    s_b = machine_cal_methods.find_earliest_machine(iter_env.lock_stations)
    if s_a == s_b:
        return 'not end'
    m_a = mission_cal_methods.find_longest_mission_for_machine(s_a)
    m_b = mission_cal_methods.find_nearest_mission_for_mission_in_other_machine(m_a, s_a, s_b)
    logger.debug("inter swap:" + s_a.idx + "-" + m_a.idx + "<->" + s_b.idx + "-" + m_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    machine_cal_methods.process_inter_swap(s_a, s_b, m_a, m_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


# check
def inter_swap_latest_station_longest_mission_all_machines(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    m_a = mission_cal_methods.find_longest_mission_for_machine(s_a)
    m_b, s_b = mission_cal_methods.find_nearest_mission_for_mission_in_other_machines(m_a, s_a, iter_env.lock_stations)
    if s_a == s_b:
        return 'not end'
    logger.debug("inter swap:" + s_a.idx + "-" + m_a.idx + "<->" + s_b.idx + "-" + m_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    machine_cal_methods.process_inter_swap(s_a, s_b, m_a, m_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'


# check
def inter_swap_latest_station_random_mission_earliest_machine(iter_env: PortEnv, buffer_flag: bool = True) -> str:
    s_a = machine_cal_methods.find_latest_machine(iter_env.lock_stations)
    s_b = machine_cal_methods.find_earliest_machine(iter_env.lock_stations)
    if s_a == s_b:
        return 'not end'
    m_a = mission_cal_methods.find_longest_mission_for_machine(s_a)
    m_b = mission_cal_methods.find_nearest_mission_for_mission_in_other_machine(m_a, s_a, s_b)
    logger.debug("inter swap:" + s_a.idx + "-" + m_a.idx + "<->" + s_b.idx + "-" + m_b.idx)
    machine_cal_methods.del_station_afterwards(iter_env, buffer_flag)
    machine_cal_methods.del_machine(s_a, buffer_flag)
    machine_cal_methods.del_machine(s_b, buffer_flag)
    machine_cal_methods.process_inter_swap(s_a, s_b, m_a, m_b, buffer_flag)
    machine_cal_methods.crossover_process_by_order(iter_env, buffer_flag)
    machine_cal_methods.yard_crane_process_by_order(iter_env, buffer_flag)
    return 'not end'
