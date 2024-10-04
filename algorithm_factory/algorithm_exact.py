#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：Port_Scheduling
@File    ：exact_result.py
@Author  ：JacQ
@Date    ：2022/2/14 17:03
"""
import itertools
import time
from copy import deepcopy

from algorithm_factory.algo_utils.machine_cal_methods import quay_crane_process_by_order, buffer_process_by_order, \
    exit_process_by_order, station_process_by_lists, crossover_process_by_order, yard_crane_process_by_order
from algorithm_factory.algo_utils.missions_sort_rules import sort_missions
from utils.log import Logger

logger = Logger().get_logger()


def get_possible_station_assign_list(mission_num, station_num):
    assign_lists = []
    station_string = ''
    for i in range(station_num):
        station_string += str(i + 1)
    for i in itertools.product(station_string, repeat=mission_num):
        assign_lists.append(i)
    logger.info("seq number" + str(len(assign_lists)))
    return assign_lists


def get_makespan(port_env, assign_list, missions_sort_rule):
    test_env = deepcopy(port_env)
    station_process_by_lists(test_env, assign_list)
    getattr(sort_missions, missions_sort_rule)(test_env.mission_list)
    crossover_process_by_order(test_env)
    getattr(sort_missions, missions_sort_rule)(test_env.mission_list)
    yard_crane_process_by_order(test_env)
    for mission in test_env.mission_list:
        mission.cal_mission_attributes()

    last_step_makespan = test_env.cal_finish_time()
    return last_step_makespan


def Exact_Method(port_env):
    start = time.time()
    port_env = deepcopy(port_env)
    missions_sort_rule = 'A_EXIT'
    logger.info("start dispatching rules.")
    quay_crane_process_by_order(port_env)
    buffer_process_by_order(port_env)
    exit_process_by_order(port_env)
    getattr(sort_missions, missions_sort_rule)(port_env.mission_list)
    assign_lists = get_possible_station_assign_list(len(port_env.mission_list),
                                                    len(port_env.lock_stations))
    g_mini_makespan = float('inf')
    g_mini_instance = 0
    g_mini_assign_list = []

    is_spark = False
    if is_spark:

        from pyspark import SparkConf, SparkContext
        conf = SparkConf()
        sc = SparkContext(conf=conf)
        parallel = 1000

        rdd = sc.parallelize(assign_lists, parallel) \
            .map(lambda x: get_makespan(port_env, x, missions_sort_rule)).cache()
        g_mini_makespan = rdd.reduce(lambda x, y: x if x < y else y)
        max_makespan = rdd.reduce(lambda x, y: x if x > y else y)
        logger.info("mini makespan: %d." % g_mini_makespan)
        logger.info("max makespan: %d." % max_makespan)
        end = time.time()
        logger.info("time: %ds." % (end - start))
    else:
        for assign_list in assign_lists:
            test_env = deepcopy(port_env)
            station_process_by_lists(test_env, assign_list)
            getattr(sort_missions, missions_sort_rule)(test_env.mission_list)
            crossover_process_by_order(test_env)
            getattr(sort_missions, missions_sort_rule)(test_env.mission_list)
            yard_crane_process_by_order(test_env)
            for mission in test_env.mission_list:
                mission.cal_mission_attributes()
            test_env.last_step_makespan = test_env.cal_finish_time()
            if test_env.last_step_makespan < g_mini_makespan:
                g_mini_makespan = test_env.last_step_makespan
                g_mini_instance = test_env
                g_mini_assign_list = assign_list
    return g_mini_makespan, g_mini_instance, g_mini_assign_list
