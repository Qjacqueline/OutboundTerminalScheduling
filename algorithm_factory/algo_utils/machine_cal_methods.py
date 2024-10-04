#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：machine_cal_methods.py
@Author  ：JacQ
@Date    ：2022/1/5 15:59
"""

import copy
import math
import random

import torch
from torch_geometric.data import Data

import conf.configs as cf
from algorithm_factory.algo_utils.missions_sort_rules import sort_missions
from common import LockStation, Crossover, YardCrane, Buffer
from common.mission import Mission
from common.port_env import PortEnv
from utils.log import Logger

logger = Logger().get_logger()


# ==================== methods utils  ====================
def find_latest_machine(machines):
    latest_finish_time = 0
    latest_machine = None
    for idx, machine in machines.items():
        if machine.process_time and machine.process_time[-1][-1] > latest_finish_time:
            latest_finish_time = machine.process_time[-1][-1]
            latest_machine = machine
    if not latest_machine:
        latest_machine = find_random_machine(machines)
    return latest_machine


def find_earliest_machine(machines):
    earliest_finish_time = float('inf')
    earliest_machine = None
    for idx, machine in machines.items():
        if machine.process_time and machine.process_time[-1][-1] < earliest_finish_time:
            earliest_finish_time = machine.process_time[-1][-1]
            earliest_machine = machine
    if not earliest_machine:
        earliest_machine = find_random_machine(machines)
    return earliest_machine


def find_random_machine(machines, r_machine=None):
    r_machines = list(machines.values())
    if r_machine:
        r_machines.remove(r_machine)
    machine = random.choice(r_machines)
    return machine


def process_init_solution_for_l2i(port_env: PortEnv, buffer_flag: bool = True):
    quay_crane_process_by_order(port_env)
    buffer_process_by_order(port_env)
    exit_process_by_order(port_env)
    getattr(sort_missions, 'FCFS')(port_env.mission_list)
    station_process_by_least_wait(port_env, buffer_flag)
    crossover_process_by_order(port_env, buffer_flag)
    yard_crane_process_by_order(port_env, buffer_flag)
    for mission in port_env.mission_list:
        mission.cal_mission_attributes(buffer_flag)


def process_init_solution_for_l2a(port_env: PortEnv, order='A_EXIT'):
    quay_crane_process_by_order(port_env)
    buffer_process_by_order(port_env)
    exit_process_by_order(port_env)
    getattr(sort_missions, order)(port_env.mission_list)


def get_least_wait_station(port_env: PortEnv, mission: Mission) -> int:
    min_wait_time: float = float('inf')
    min_station: LockStation = list(port_env.lock_stations.items())[0][1]
    for i in range(port_env.ls_num):
        curr_station = list(port_env.lock_stations.values())[i]
        transfer_time_station = curr_station.distance_to_exit / mission.vehicle_speed
        arrive_time_station = mission.total_process_time + mission.machine_start_time[0] + transfer_time_station
        if any(curr_station.process_time):
            wait_time = max(curr_station.process_time[-1][-1] - arrive_time_station, 0)
        else:
            wait_time = 0
        if wait_time < min_wait_time:
            min_wait_time = wait_time
            min_station = curr_station
    return int(min_station.idx[-1]) - 1


def get_least_num_station(port_env: PortEnv) -> int:
    min_num: int = 100000
    min_station: LockStation = list(port_env.lock_stations.items())[0][1]
    for i in range(port_env.ls_num):
        curr_station = list(port_env.lock_stations.values())[i]
        curr_station_num = len(curr_station.mission_list)
        if curr_station_num < min_num:
            min_num = curr_station_num
            min_station = curr_station
    return int(min_station.idx[-1]) - 1


def get_matched_crossover(mission: Mission):
    crossover_idx = 0
    block = mission.yard_block_loc[0]
    if block[0] == 'A':
        crossover_idx = 'CO1'
    if block[0] == 'B':
        crossover_idx = 'CO2'
    if block[0] == 'C':
        crossover_idx = 'CO3'
    return crossover_idx


def get_est_arrive_crossover_time(port_env: PortEnv, mission: Mission):
    min_tmp_time = float('Inf')
    for i in range(port_env.ls_num):
        station = list(port_env.lock_stations.values())[i]
        arrive_station_time = mission.machine_start_time[1] + station.distance_to_exit / (sum(cf.VEHICLE_SPEED) / 2.0)
        transfer_time_station_crossover = port_env.ls_to_co_matrix[int(station.idx[-1]) - 1][
                                              int(mission.crossover_id[-1]) - 1] / (sum(cf.VEHICLE_SPEED) / 2.0)
        if any(station.process_time) and arrive_station_time < station.process_time[-1][-1]:
            tmp_time = station.process_time[-1][-1] + mission.station_process_time + \
                       transfer_time_station_crossover + 30
        else:
            tmp_time = arrive_station_time + mission.station_process_time + transfer_time_station_crossover
        if min_tmp_time > tmp_time:
            min_tmp_time = tmp_time
    return min_tmp_time


def get_next_job_at_quay_cranes(port_env: PortEnv, curr_time: list):
    min_time = float('INF')
    min_idx = None
    for i in range(len(port_env.quay_cranes)):
        tmp_time = port_env.quay_cranes['QC' + str(i + 1)].time_to_exit + curr_time[i] + 60
        if tmp_time < min_time and len(port_env.quay_cranes['QC' + str(i + 1)].mission_list) < len(
                port_env.quay_cranes['QC' + str(i + 1)].missions):
            min_time = tmp_time
            min_idx = i
    return list(port_env.quay_cranes['QC' + str(min_idx + 1)].missions.values())[
        len(port_env.quay_cranes['QC' + str(min_idx + 1)].mission_list)]


def get_cur_time_status(port_env: PortEnv, cur_time: float):
    qc_ls = {'QC1': [], 'QC2': [], 'QC3': []}
    qc_ls_ls = []
    ls_ls = {'S1': [], 'S2': [], 'S3': [], 'S4': []}
    ls_co_ls = []
    co_ls = {'CO1': [], 'CO2': [], 'CO3': []}
    co_yc_ls = []
    yc_ls = {}
    for key in port_env.yard_cranes_set:
        yc_ls.setdefault(key, [])
    f_ls = []

    for mission in port_env.mission_list:
        if mission.machine_start_time[0] >= cur_time:
            qc_ls[mission.quay_crane_id].append(mission)
        elif mission.machine_start_time[2] > cur_time:
            qc_ls_ls.append(mission)
        elif mission.machine_start_time[4] + mission.machine_process_time[4] > cur_time:
            ls_ls[mission.machine_list[4]].append(mission)
        elif mission.machine_start_time[5] > cur_time:
            ls_co_ls.append(mission)
        elif mission.machine_start_time[5] + mission.machine_process_time[5] + mission.machine_process_time[6] \
                > cur_time:
            co_ls[mission.machine_list[6]].append(mission)
        elif mission.machine_start_time[7] > cur_time:
            co_yc_ls.append(mission)
        elif mission.total_process_time + mission.machine_start_time[0] > cur_time:
            yc_ls[mission.machine_list[8][2:]].append(mission)
        else:
            f_ls.append(mission)

    return qc_ls, qc_ls_ls, ls_ls, ls_co_ls, co_ls, co_yc_ls, yc_ls, f_ls


def get_cur_time_status_v2(port_env: PortEnv, cur_time: float):
    qc_ls, ls_ls, co_ls, yc_ls = {}, {}, {}, {}
    for i in range(port_env.qc_num):
        qc_ls['QC' + str(i + 1)] = []
    for i in range(port_env.ls_num):
        ls_ls['S' + str(i + 1)] = []
    for i in range(port_env.is_num):
        co_ls['CO' + str(i + 1)] = []
    qc_ls_ls, ls_co_ls, co_yc_ls, f_ls = [], [], [], []
    for key in port_env.yard_cranes_set:
        yc_ls.setdefault(key, [])
    for mission in port_env.mission_list:
        if mission.machine_start_time[1] >= cur_time or len(mission.machine_start_time) == 2:
            qc_ls[mission.quay_crane_id].append(mission)
        elif mission.machine_start_time[2] > cur_time:
            ls_ls[mission.machine_list[4]].append(mission)
        elif mission.machine_start_time[4] + mission.machine_process_time[4] > cur_time:
            ls_ls[mission.machine_list[4]].append(mission)
        elif mission.machine_start_time[5] > cur_time:
            co_ls[mission.machine_list[6]].append(mission)
        elif mission.machine_start_time[6] + mission.machine_process_time[6] > cur_time:
            co_ls[mission.machine_list[6]].append(mission)
        elif mission.machine_start_time[7] > cur_time:
            yc_ls[mission.machine_list[8][2:]].append(mission)
        elif mission.machine_start_time[8] + mission.machine_process_time[8] > cur_time:
            yc_ls[mission.machine_list[8][2:]].append(mission)
        else:
            f_ls.append(mission)

    return qc_ls, qc_ls_ls, ls_ls, ls_co_ls, co_ls, co_yc_ls, yc_ls, f_ls


def split_integer(m, n):
    assert n > 0
    quotient = int(m / n)
    remainder = m % n
    if remainder > 0:
        return [quotient] * (n - remainder) + [quotient + 1] * remainder
    if remainder < 0:
        return [quotient - 1] * -remainder + [quotient] * (n + remainder)
    return [quotient] * n


def generate_instance_type(inst_type, mission_num=-1):
    if inst_type == 'A_t' or inst_type == 'A2_t' or inst_type == 'A2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 1, 2, 1, 2, cf.MISSION_NUM
    elif inst_type == 'B_t' or inst_type == 'B2_t' or inst_type == 'B2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 2, 2, 2, 4, cf.MISSION_NUM
    elif inst_type == 'C_t' or inst_type == 'C2_t' or inst_type == 'C2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 2, 2, 3, cf.MISSION_NUM
    elif inst_type == 'D_t' or inst_type == 'D2_t' or inst_type == 'D2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 3, 1, 3, cf.MISSION_NUM
    elif inst_type == 'E_t' or inst_type == 'E2_t' or inst_type == 'E2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 4, 2, 4, cf.MISSION_NUM
    elif inst_type == 'F_t' or inst_type == 'F2_t' or inst_type == 'F2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 5, 2, 3, cf.MISSION_NUM
    elif inst_type == 'G_t' or inst_type == 'G2_t' or inst_type == 'G2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 4, 4, 3, 6, cf.MISSION_NUM
    elif inst_type == 'H_t' or inst_type == 'H2_t' or inst_type == 'H2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 5, 5, 3, 7, cf.MISSION_NUM
    elif inst_type == 'Z_t' or inst_type == 'Z2_t' or inst_type == 'Z2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 6, 5, 3, 8, cf.MISSION_NUM
    elif inst_type == 'I_t' or inst_type == 'I2_t' or inst_type == 'I2_1000':
        qc_num, ls_num, is_num, yc_num, m_num = 6, 5, 3, 8, cf.MISSION_NUM
    elif inst_type == 'Z0_t':
        qc_num, ls_num, is_num, yc_num, m_num = 6, 5, 3, 8, mission_num
    elif inst_type == 'A' or inst_type == 'A2':
        qc_num, ls_num, is_num, yc_num, m_num = 1, 2, 1, 2, 100
    elif inst_type == 'B' or inst_type == 'B2':
        qc_num, ls_num, is_num, yc_num, m_num = 2, 2, 2, 4, 100
    elif inst_type == 'C' or inst_type == 'C2':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 2, 2, 3, cf.MISSION_NUM
    elif inst_type == 'D' or inst_type == 'D2':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 3, 1, 3, 100
    elif inst_type == 'E' or inst_type == 'E2':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 4, 2, 4, 100
    elif inst_type == 'F' or inst_type == 'F2':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 5, 2, 3, 100
    elif inst_type == 'G' or inst_type == 'G2':
        qc_num, ls_num, is_num, yc_num, m_num = 4, 4, 3, 6, 100
    elif inst_type == 'H' or inst_type == 'H2':
        qc_num, ls_num, is_num, yc_num, m_num = 5, 5, 3, 7, 100
    elif inst_type == 'Z' or inst_type == 'Z2':
        qc_num, ls_num, is_num, yc_num, m_num = 6, 5, 3, 8, 100
    elif inst_type == 'CA':
        qc_num, ls_num, is_num, yc_num, m_num = 6, 5, 3, 8, 382
    elif inst_type == 'QC-1':
        qc_num, ls_num, is_num, yc_num, m_num = 2, 2, 2, 3, cf.MISSION_NUM
    elif inst_type == 'QC+1':
        qc_num, ls_num, is_num, yc_num, m_num = 4, 2, 2, 3, cf.MISSION_NUM
    elif inst_type == 'LS-1':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 1, 2, 3, cf.MISSION_NUM
    elif inst_type == 'LS+1':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 3, 2, 3, cf.MISSION_NUM
    elif inst_type == 'IS-1':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 2, 1, 3, cf.MISSION_NUM
    elif inst_type == 'IS+1':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 2, 3, 3, cf.MISSION_NUM
    elif inst_type == 'YC-1':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 2, 2, 2, cf.MISSION_NUM
    elif inst_type == 'YC+1':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 2, 2, 4, cf.MISSION_NUM
    elif inst_type == 'low':
        qc_num, ls_num, is_num, yc_num, m_num = 3, 2, 2, 3, 600
    elif inst_type == 'medium':
        qc_num, ls_num, is_num, yc_num, m_num = 5, 4, 2, 5, 1350
    elif inst_type == 'heavy':
        qc_num, ls_num, is_num, yc_num, m_num = 8, 6, 3, 8, 4000
    else:
        qc_num, ls_num, is_num, yc_num, m_num = -1, -1, -1, -1, -1
    return qc_num, ls_num, is_num, yc_num, m_num


def generate_yard_blocks_set(is_num, yc_num):
    cur_yar_blocks = []
    if is_num == 3:
        is_ls = [int(yc_num / 3), int((yc_num - int(yc_num / 3)) / 2),
                 yc_num - int(yc_num / 3) - int((yc_num - int(yc_num / 3)) / 2)]
        random.shuffle(is_ls)
        cur_yar_blocks.extend(random.sample(['A1', 'A2', 'A3', 'A4'], is_ls[0]))
        cur_yar_blocks.extend(random.sample(['B1', 'B2', 'B3', 'B4'], is_ls[1]))
        cur_yar_blocks.extend(random.sample(['C1', 'C2', 'C3', 'C4'], is_ls[2]))
    if is_num == 2:
        is_ls = [int(yc_num / 2), yc_num - int(yc_num / 2), 0]
        cur_yar_blocks.extend(random.sample(['A1', 'A2', 'A3', 'A4'], is_ls[0]))
        cur_yar_blocks.extend(random.sample(['B1', 'B2', 'B3', 'B4'], is_ls[1]))
        cur_yar_blocks.extend(random.sample(['C1', 'C2', 'C3', 'C4'], is_ls[2]))
    if is_num == 1:
        is_ls = [yc_num, 0, 0]
        cur_yar_blocks.extend(random.sample(['A1', 'A2', 'A3', 'A4'], is_ls[0]))
        cur_yar_blocks.extend(random.sample(['B1', 'B2', 'B3', 'B4'], is_ls[1]))
        cur_yar_blocks.extend(random.sample(['C1', 'C2', 'C3', 'C4'], is_ls[2]))
    return cur_yar_blocks


def cal_congestion(port_env: PortEnv):
    congestion_time = 0
    for mission in port_env.mission_list:
        congestion_time += sum(mission.waiting_time[0:3])
    return congestion_time


def cal_LB1(port_env: PortEnv):
    m_m, m_2_3_4, m_ls, m_co, m_1_2_3, m_1, m_3_4, m_1_2, m_4 = float('inf'), float('inf'), float('inf'), float(
        'inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf')
    lb_qc, lb_ls, lb_co, lb_yc = 0, 0, 0, 0
    for mission in port_env.mission_list:
        _, min_distance = find_least_distance_station(port_env, mission)
        lb_ls += mission.station_process_time
        t_m_qc = mission.station_process_time + mission.intersection_process_time + \
                 mission.yard_crane_process_time + \
                 abs(cf.SLOT_NUM_Y - mission.yard_block_loc[2]) * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y * 2 + \
                 min_distance / mission.vehicle_speed + mission.transfer_time_c2y
        t_m_yc = mission.release_time / 2.0 + cf.BUFFER_PROCESS_TIME + mission.station_process_time + \
                 mission.intersection_process_time + min_distance / mission.vehicle_speed + mission.transfer_time_c2y
        t_m_1 = mission.release_time / 2.0 + cf.BUFFER_PROCESS_TIME + \
                port_env.quay_cranes[mission.quay_crane_id].time_to_exit + mission.transfer_time_e2s_min
        t_m_3_4 = mission.intersection_process_time + \
                  mission.yard_crane_process_time + \
                  abs(cf.SLOT_NUM_Y - mission.yard_block_loc[2]) * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y * 2 + \
                  mission.transfer_time_s2c_min + mission.transfer_time_c2y
        t_m_1_2 = mission.release_time / 2.0 + cf.BUFFER_PROCESS_TIME + mission.station_process_time + \
                  min_distance
        t_m_4 = mission.yard_crane_process_time + \
                abs(cf.SLOT_NUM_Y - mission.yard_block_loc[2]) * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y * 2 + \
                mission.transfer_time_c2y
        if t_m_qc < m_2_3_4:
            m_2_3_4 = t_m_qc
        if t_m_yc < m_1_2_3:
            m_1_2_3 = t_m_yc
        if t_m_1 < m_1:
            m_1 = t_m_1
        if t_m_3_4 < m_3_4:
            m_3_4 = t_m_3_4
        if t_m_1_2 < m_1_2:
            m_1_2 = t_m_1_2
        if t_m_4 < m_4:
            m_4 = t_m_4
        if mission.release_time / 2.0 + cf.BUFFER_PROCESS_TIME > lb_qc:
            lb_qc = mission.release_time / 2.0 + cf.BUFFER_PROCESS_TIME
    lb_ls = lb_ls / len(port_env.lock_stations)
    for co in port_env.crossovers.values():
        t_lb_co = 0
        for mission in co.mission_list:
            t_lb_co += mission.intersection_process_time
        if t_lb_co > lb_co:
            lb_co = t_lb_co
    for yc_idx in port_env.yard_cranes_set:
        yc = port_env.yard_cranes['YC' + yc_idx]
        max_x_ls = [mission.yard_block_loc[1] for mission in yc.mission_list]
        yc_pt_ls = [mission.yard_crane_process_time + abs(cf.SLOT_NUM_Y - mission.yard_block_loc[2])
                    * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y * 2 for mission in yc.mission_list]
        t_lb_yc = sum(yc_pt_ls) + max(max_x_ls) * cf.SLOT_LENGTH / cf.YARDCRANE_SPEED_X
        if t_lb_yc > lb_yc:
            lb_yc = t_lb_yc
    lb1 = max((lb_qc + m_2_3_4), (lb_ls + m_1 + m_3_4), (lb_co + m_1_2 + m_4), (m_1_2_3 + lb_yc))
    # print("lB1:" + str(lb1))
    return lb1


def cal_LB2(port_env: PortEnv):
    lb2 = 0
    r_lb2 = {}
    for mission in port_env.mission_list:
        _, min_distance = find_least_distance_station(port_env, mission)
        t_lb2 = mission.release_time / 2.0 \
                + cf.BUFFER_PROCESS_TIME \
                + mission.station_process_time \
                + mission.intersection_process_time \
                + mission.yard_crane_process_time \
                + abs(cf.SLOT_NUM_Y - mission.yard_block_loc[2]) * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y * 2 \
                + mission.transfer_time_c2y \
                + min_distance / mission.vehicle_speed
        if t_lb2 > lb2:
            lb2 = t_lb2
        r_lb2[mission.idx] = t_lb2
    # print("lB2:" + str(lb2))
    return lb2, r_lb2


def cal_LB3(port_env: PortEnv, r_lb: dict):
    r_lb2 = copy.deepcopy(r_lb)
    yc_dict = {}
    for yc_idx in port_env.yard_cranes_set:
        yc_dict.setdefault(yc_idx, [])
    t_yc_ls = copy.deepcopy(yc_dict)
    getattr(sort_missions, "A_EXIT")(port_env.mission_list)
    for mission in port_env.mission_list:
        ls = t_yc_ls.get(mission.yard_block_loc[0])
        if any(ls):
            p_mission = ls[-1]
            r_lb2[mission.idx] = max(r_lb2[mission.idx], r_lb2[p_mission] + mission.yard_crane_process_time + abs(
                cf.SLOT_NUM_Y - mission.yard_block_loc[2]) * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y * 2)
        ls.append(mission.idx)
        t_yc_ls[mission.yard_block_loc[0]] = ls
    # print("LB3:" + str(max(list(r_lb2.values()))))
    return max(list(r_lb2.values()))


# ==================== machine process missions  ====================
def quay_crane_process_by_order(port_env):
    for qc in port_env.quay_cranes.values():
        for mission in qc.missions.values():
            port_env.mission_list.append(mission)
            # qc.mission_list.append(mission)


def buffer_process_by_order(port_env):
    for qc, buffer in zip(port_env.quay_cranes.values(), port_env.buffers.values()):
        for mission in qc.missions.values():
            buffer_process_one_order(mission, buffer)


def buffer_process_one_order(mission: Mission, buffer: Buffer):
    start_time_buffer = mission.release_time / 2
    process_time_buffer = buffer.handling_time
    end_time_buffer = start_time_buffer + process_time_buffer
    buffer.mission_list.append(mission)
    buffer.process_time.append([start_time_buffer, process_time_buffer, end_time_buffer])
    mission.total_process_time += process_time_buffer
    mission.machine_list.append(buffer.idx)
    mission.machine_start_time.append(start_time_buffer)
    mission.machine_process_time.append(process_time_buffer)
    mission.stage = 2


def exit_process_by_order(port_env):
    for mission in port_env.mission_list:
        qc_loc = port_env.quay_cranes[mission.quay_crane_id].location
        exit_loc = cf.QUAY_EXIT
        transfer_time_station = (abs(qc_loc[0] - exit_loc[0]) + abs(qc_loc[1] - exit_loc[1])) / mission.vehicle_speed
        mission.total_process_time += transfer_time_station
        mission.machine_list.append('a_exit')
        mission.machine_process_time.append(0)
        mission.machine_start_time.append(mission.total_process_time + mission.machine_start_time[0])
        mission.stage = 3


def station_process_by_random_no_sort(port_env, buffer_flag=False):
    for mission in port_env.mission_list:
        r = random.randint(0, len(port_env.lock_stations) - 1)
        curr_station = list(port_env.lock_stations.items())[r][1]
        assign_mission_to_station(mission, curr_station, buffer_flag)


def station_process_by_random(port_env, buffer_flag=False):
    station_assign_dict = {}
    for station_idx in port_env.lock_stations.keys():
        station_assign_dict.setdefault(station_idx, [])
    for mission in port_env.mission_list:
        r = random.randint(0, port_env.ls_num - 1)
        curr_station = list(port_env.lock_stations.items())[r][1]
        station_assign_dict[curr_station.idx].append(mission)
    for station_idx in station_assign_dict.keys():
        getattr(sort_missions, "A_STATION")(station_assign_dict[station_idx],
                                            port_env.lock_stations[station_idx])
        for mission in station_assign_dict[station_idx]:
            assign_mission_to_station(mission, port_env.lock_stations[station_idx], buffer_flag)


def station_process_by_fixed_order(port_env, order, buffer_flag=False):
    station_assign_dict = {}
    for station_idx in port_env.lock_stations.keys():
        station_assign_dict.setdefault(station_idx, [])

    for mission in port_env.mission_list:
        r = order[int(mission.idx[1:]) - 1]
        curr_station = list(port_env.lock_stations.items())[r][1]
        station_assign_dict[curr_station.idx].append(mission)

    for station_idx in station_assign_dict.keys():
        getattr(sort_missions, "A_STATION")(station_assign_dict[station_idx],
                                            port_env.lock_stations[station_idx])
        for mission in station_assign_dict[station_idx]:
            assign_mission_to_station(mission, port_env.lock_stations[station_idx], buffer_flag)


def station_process_by_least_wait(port_env, buffer_flag=True):
    station_assign_dict = {}
    for station_idx in port_env.lock_stations.keys():
        station_assign_dict.setdefault(station_idx, [])
    for mission in port_env.mission_list:
        min_station = find_min_wait_station(port_env, mission)
        assign_mission_to_station(mission, min_station, buffer_flag)
        station_assign_dict[min_station.idx].append(mission)


def find_min_wait_station(port_env: PortEnv, mission: Mission):
    min_wait_time = float('inf')
    min_station = list(port_env.lock_stations.items())[0][1]
    for i in range(port_env.ls_num):
        curr_station = list(port_env.lock_stations.values())[i]
        transfer_time_station = curr_station.distance_to_exit / mission.vehicle_speed
        arrive_time_station = mission.total_process_time + mission.machine_start_time[0] + transfer_time_station
        if any(curr_station.process_time):
            wait_time = max(curr_station.process_time[-1][-1] - arrive_time_station, 0)
        else:
            wait_time = 0
        if wait_time < min_wait_time:
            min_wait_time = wait_time
            min_station = curr_station
    return min_station


def station_process_by_least_mission_num(port_env: PortEnv, buffer_flag=False):
    station_assign_dict = {}
    for i in range(port_env.ls_num):
        station_idx = list(port_env.lock_stations)[i]
        station_assign_dict.setdefault(station_idx, [])
    getattr(sort_missions, 'A_EXIT')(port_env.mission_list)
    for mission in port_env.mission_list:
        min_mission_num = 100000
        min_station_idx = 'S1'
        for station_idx in station_assign_dict.keys():
            if any(station_assign_dict[station_idx]):
                mission_num = len(station_assign_dict[station_idx])
            else:
                mission_num = 0
            if mission_num < min_mission_num:
                min_mission_num = mission_num
                min_station_idx = station_idx
        station_assign_dict[min_station_idx].append(mission)
    for station_idx in station_assign_dict.keys():
        getattr(sort_missions, "A_STATION")(station_assign_dict[station_idx],
                                            port_env.lock_stations[station_idx])
        for mission in station_assign_dict[station_idx]:
            assign_mission_to_station(mission, port_env.lock_stations[station_idx], buffer_flag)

def find_least_mission_station(port_env: PortEnv):
    station_assign_dict = {}
    for i in range(port_env.ls_num):
        station_idx = list(port_env.lock_stations)[i]
        station_assign_dict.setdefault(station_idx, [])
    min_mission_num = 100000
    min_station_idx = 'S1'
    for station_idx in station_assign_dict.keys():
        if any(station_assign_dict[station_idx]):
            mission_num = len(station_assign_dict[station_idx])
        else:
            mission_num = 0
        if mission_num < min_mission_num:
            min_mission_num = mission_num
            min_station_idx = station_idx
    return int(min_station_idx[1:])-1

def match_mission_crossover(crossovers, mission):
    curr_crossover = None
    block_column = mission.yard_block_loc[0][0]
    if block_column == 'A':
        curr_crossover = crossovers['CO1']
    if block_column == 'B':
        curr_crossover = crossovers['CO2']
    if block_column == 'C':
        curr_crossover = crossovers['CO3']
    return curr_crossover


def match_mission_yard_crane_num(mission, env: PortEnv):
    yc_idx = env.qc_num + env.ls_num + env.ls_num - 1
    block_column = mission.yard_block_loc[0][0]
    if block_column == 'A':  # YC 10-13
        yc_idx = yc_idx + int(mission.yard_block_loc[0][-1])
    if block_column == 'B':  # YC 14-17
        yc_idx = yc_idx + 4 + int(mission.yard_block_loc[0][-1])
    if block_column == 'C':  # YC 18-21
        yc_idx = yc_idx + 8 + int(mission.yard_block_loc[0][-1])
    return yc_idx


def del_station_afterwards(port_env: PortEnv, buffer_flag, step_number=None, released_mission_ls=None):
    if step_number is None:
        step_number = port_env.J_num_all
    if buffer_flag:
        crossover_stage = 5
    else:
        crossover_stage = 4
    if released_mission_ls is not None:
        ls = released_mission_ls
    else:
        ls = port_env.mission_list
    for i in range(step_number):
        mission = ls[i]
        del mission.machine_list[crossover_stage:]
        del mission.machine_process_time[crossover_stage:]
        del mission.machine_start_time[crossover_stage:]
        mission.total_process_time = mission.machine_start_time[crossover_stage - 1] + mission.machine_process_time[
            crossover_stage - 1] - mission.machine_start_time[0]
    for machine in port_env.crossovers.values():
        machine.mission_list = []
        machine.process_time = []
    for machine in port_env.yard_cranes.values():
        machine.mission_list = []
        machine.process_time = []


def del_machine(machine, buffer_flag=False):
    if machine.idx[0] == 'S':
        stage = 2
        machine.process_time = []
        if buffer_flag:
            machine.whole_occupy_time = []
            for idx, station_buffer in machine.lock_station_buffers.items():
                station_buffer.process_time = []
                station_buffer.mission_list = []
        for mission in machine.mission_list:
            mission.total_process_time = mission.machine_start_time[stage - 1] - mission.machine_start_time[0]
            del mission.machine_list[stage:]
            del mission.machine_process_time[stage:]
            del mission.machine_start_time[stage:]
            if buffer_flag:
                mission.stage = 5
            else:
                mission.stage = 4
    if machine.idx[0] == 'C':
        stage = 5
        for mission in machine.mission_list:
            mission.total_process_time = mission.machine_start_time[stage - 1] - mission.machine_start_time[0]
            del mission.machine_list[stage:]
            del mission.machine_process_time[stage:]
            del mission.machine_start_time[stage:]
        machine.process_time = []
        machine.mission_list = []

    if machine.idx[0] == 'Y':
        stage = 7
        for mission in machine.mission_list:
            mission.total_process_time = mission.machine_start_time[stage - 1] + mission.machine_process_time[
                stage - 1] - mission.machine_start_time[0]
            del mission.machine_list[stage:]
            del mission.machine_process_time[stage:]
            del mission.machine_start_time[stage:]
        machine.process_time = []
        machine.mission_list = []


def cal_yard_missions_matrix(iter_solution):
    exist_yard_crane_info = {}
    adj_matrix = [[0 for _ in range(len(iter_solution.mission_list))] for _ in
                  range(len(iter_solution.mission_list))]
    for mission in iter_solution.mission_list:
        exist_yard_crane_info.setdefault(mission.yard_block_loc[0], []).append(mission)
    for yard_crane_idx, missions in exist_yard_crane_info.items():
        for i in range(len(missions)):
            for j in range(i, len(missions)):
                a_yard_loc = missions[i].yard_block_loc
                b_yard_loc = missions[j].yard_block_loc
                adj_matrix[i][j] = abs(
                    a_yard_loc[1] - b_yard_loc[1]) * cf.SLOT_LENGTH / cf.YARDCRANE_SPEED_X + abs(
                    a_yard_loc[2] - b_yard_loc[2]) * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y
                adj_matrix[j][i] = adj_matrix[i][j]
    return adj_matrix


def output_solution(solution: PortEnv) -> dict:
    mission_station_dict = dict()
    for mission in solution.mission_list:
        mission_station_dict[mission.idx] = int(mission.machine_list[4][-1]) - 1

    return mission_station_dict


# ==================== machine process missions  ====================
def quay_crane_release_mission(port_env: PortEnv, mission: Mission):
    port_env.quay_cranes[mission.quay_crane_id].mission_list.append(mission)


def find_least_distance_station(port_env, mission):
    min_distance = 10000
    min_station = list(port_env.lock_stations.items())[0][1]
    curr_crossover = port_env.crossovers[mission.crossover_id]
    for i in range(port_env.ls_num):
        curr_station = list(port_env.lock_stations.values())[i]
        curr_station_loc = curr_station.location
        distance = abs(port_env.quay_cranes[mission.quay_crane_id].location[0] - curr_station_loc[0]) + \
                   abs(port_env.quay_cranes[mission.quay_crane_id].location[1] - curr_station_loc[1]) + \
                   abs(curr_crossover.location[0] - curr_station_loc[0]) + \
                   abs(curr_crossover.location[1] - curr_station_loc[1])
        if distance < min_distance:
            min_distance = distance
            min_station = curr_station
    return min_station, min_distance


def station_process_by_least_distance(port_env, buffer_flag=False):
    station_assign_dict = {}
    for i in range(port_env.ls_num):
        station_idx = list(port_env.lock_stations)[i]
        station_assign_dict.setdefault(station_idx, [])
    for mission in port_env.mission_list:
        min_station, min_distance = find_least_distance_station(port_env, mission)
        station_assign_dict[min_station.idx].append(mission)
    for station_idx in station_assign_dict.keys():
        getattr(sort_missions, "A_STATION")(station_assign_dict[station_idx],
                                            port_env.lock_stations[station_idx])
        for mission in station_assign_dict[station_idx]:
            assign_mission_to_station(mission, port_env.lock_stations[station_idx], buffer_flag)


def station_process_by_lists(test_env, assign_list, buffer_flag=False):
    for i in range(len(test_env.mission_list)):
        r = int(assign_list[i]) - 1
        curr_station = list(test_env.lock_stations.items())[r][1]
        assign_mission_to_station(test_env.mission_list[i], curr_station, buffer_flag)


def assign_mission_to_least_wait_station_buffer(curr_station):
    min_wait_time = float("INF")
    min_wait_station_buffer = None
    for idx, cur_station_buffer in curr_station.lock_station_buffers.items():
        if any(cur_station_buffer.process_time):
            if min_wait_time >= cur_station_buffer.process_time[-1][-1]:
                min_wait_time = cur_station_buffer.process_time[-1][-1]
                min_wait_station_buffer = cur_station_buffer
        else:
            min_wait_station_buffer = cur_station_buffer
            break
    return min_wait_station_buffer


def assign_mission_to_station(mission, curr_station, buffer_flag=False):
    if buffer_flag:
        transfer_time_to_station = curr_station.distance_to_exit / mission.vehicle_speed
        mission_arrive_time_station = mission.total_process_time + mission.machine_start_time[
            0] + transfer_time_to_station
        if any(curr_station.whole_occupy_time) and curr_station.whole_occupy_time[-1][-1] > mission_arrive_time_station:
            min_wait_station_buffer = assign_mission_to_least_wait_station_buffer(curr_station)
            end_time_at_buffer = curr_station.process_time[-1][-1]
            wait_time_at_buffer = end_time_at_buffer - mission_arrive_time_station
            station_start_time = end_time_at_buffer + min_wait_station_buffer.wait_time_delay
            min_wait_station_buffer.mission_list.append(mission)
            min_wait_station_buffer.process_time.append(
                [mission_arrive_time_station, wait_time_at_buffer, end_time_at_buffer])
            curr_station.mission_list.append(mission)
            curr_station.process_time.append(
                [station_start_time, mission.station_process_time, station_start_time + mission.station_process_time])
            curr_station.whole_occupy_time.append(
                [mission_arrive_time_station,
                 station_start_time + mission.station_process_time - mission_arrive_time_station,
                 station_start_time + mission.station_process_time])
            mission.machine_list.append('a_station')
            mission.machine_start_time.append(mission_arrive_time_station)
            mission.machine_process_time.append(
                station_start_time - mission_arrive_time_station)
            mission.machine_list.append(min_wait_station_buffer.idx)
            mission.machine_start_time.append(mission_arrive_time_station)
            mission.machine_process_time.append(wait_time_at_buffer)

            mission.machine_list.append(curr_station.idx)
            mission.machine_start_time.append(station_start_time)
            mission.machine_process_time.append(mission.station_process_time)

            mission.total_process_time = station_start_time + mission.station_process_time \
                                         - mission.machine_start_time[0]

        else:
            curr_station.mission_list.append(mission)
            curr_station.process_time.append(
                [mission_arrive_time_station, mission.station_process_time,
                 mission_arrive_time_station + mission.station_process_time])
            curr_station.whole_occupy_time.append(
                [mission_arrive_time_station, mission.station_process_time,
                 mission_arrive_time_station + mission.station_process_time])
            mission.machine_list.append('a_station')
            mission.machine_start_time.append(mission_arrive_time_station)
            mission.machine_process_time.append(0)

            mission.machine_list.append("LS0")
            mission.machine_start_time.append(mission_arrive_time_station)
            mission.machine_process_time.append(0)

            mission.machine_list.append(curr_station.idx)
            mission.machine_start_time.append(mission_arrive_time_station)
            mission.machine_process_time.append(mission.station_process_time)

            mission.total_process_time += transfer_time_to_station + mission.station_process_time
        mission.stage = 5
    else:
        transfer_time_to_station = (abs(curr_station.location[0] - cf.QUAY_EXIT[0]) + abs(
            curr_station.location[1] - cf.QUAY_EXIT[1])) / mission.vehicle_speed
        mission_arrive_time_station = mission.total_process_time + mission.machine_start_time[
            0] + transfer_time_to_station
        mission_handling_time_station = mission.station_process_time
        if any(curr_station.process_time):
            mission_start_time_station = max(mission_arrive_time_station, curr_station.process_time[-1][-1])
        else:
            mission_start_time_station = mission_arrive_time_station
        end_time_station = mission_start_time_station + mission_handling_time_station

        curr_station.mission_list.append(mission)
        curr_station.process_time.append([mission_start_time_station, mission_handling_time_station, end_time_station])

        mission.machine_list.append('a_station')
        mission.machine_start_time.append(mission_arrive_time_station)
        mission.machine_process_time.append(mission_start_time_station - mission_arrive_time_station)

        mission.machine_list.append(curr_station.idx)
        mission.machine_start_time.append(mission_start_time_station)
        mission.machine_process_time.append(mission_handling_time_station)

        mission.total_process_time += transfer_time_to_station + mission_start_time_station - \
                                      mission_arrive_time_station + mission_handling_time_station
        mission.stage = 4


def crossover_process_by_order(port_env, buffer_flag=False, step_number=None, released_mission_ls=None):
    crossover_assign_dict = {}
    for crossover_idx in port_env.crossovers.keys():
        crossover_assign_dict.setdefault(crossover_idx, [])
    if released_mission_ls is not None:
        ls = released_mission_ls
    else:
        ls = port_env.mission_list
    if step_number is None:
        step_number = port_env.J_num_all
    for i in range(step_number):
        mission = ls[i]
        crossover = port_env.crossovers[mission.crossover_id]
        crossover_assign_dict[crossover.idx].append(mission)
    for crossover_idx in crossover_assign_dict.keys():
        getattr(sort_missions, "A_CROSSOVER_UA")(crossover_assign_dict[crossover_idx],
                                                 [i[int(crossover_idx[-1]) - 1] for i in
                                                  port_env.ls_to_co_matrix])
        for tmp_mission in crossover_assign_dict[crossover_idx]:
            assign_mission_to_crossover(port_env.lock_stations, port_env.crossovers, tmp_mission, buffer_flag)


def assign_mission_to_crossover(lock_stations, crossovers, mission, buffer_flag=False):
    if buffer_flag:
        mission.stage = 5
    else:
        mission.stage = 4
    curr_crossover = crossovers[mission.crossover_id]
    temp_station_loc = lock_stations[mission.machine_list[mission.stage - 1]].location
    transfer_time_crossover = (abs(curr_crossover.location[0] - temp_station_loc[0]) + abs(
        curr_crossover.location[1] - temp_station_loc[1])) / mission.vehicle_speed
    arrive_time_crossover = mission.total_process_time + mission.machine_start_time[0] + transfer_time_crossover
    handling_time_crossover = mission.intersection_process_time
    if any(curr_crossover.process_time):
        start_time_crossover = max(arrive_time_crossover, curr_crossover.process_time[-1][-1])
    else:
        start_time_crossover = arrive_time_crossover
    end_time_crossover = start_time_crossover + handling_time_crossover

    curr_crossover.mission_list.append(mission)
    curr_crossover.process_time.append([start_time_crossover, handling_time_crossover, end_time_crossover])

    mission.machine_list.append('a_crossover')
    mission.machine_start_time.append(arrive_time_crossover)
    mission.machine_process_time.append(start_time_crossover - arrive_time_crossover)

    mission.machine_list.append(curr_crossover.idx)
    mission.machine_start_time.append(start_time_crossover)
    mission.machine_process_time.append(handling_time_crossover)
    mission.total_process_time += transfer_time_crossover + handling_time_crossover + start_time_crossover \
                                  - arrive_time_crossover
    if buffer_flag:
        mission.stage = 7
    else:
        mission.stage = 6


def yard_crane_process_by_order(port_env, buffer_flag=False, step_number=None, released_mission_ls=None):
    yard_crane_assign_dict = {}
    for yard_crane_idx in port_env.yard_cranes.keys():
        yard_crane_assign_dict.setdefault(yard_crane_idx, [])
    if released_mission_ls is not None:
        ls = released_mission_ls
    else:
        ls = port_env.mission_list
    if step_number is None:
        step_number = port_env.J_num_all
    for i in range(step_number):
        mission = ls[i]
        yard_crane_assign_dict['YC' + mission.yard_block_loc[0]].append(mission)
    for yard_crane_idx in yard_crane_assign_dict:
        if any(yard_crane_assign_dict[yard_crane_idx]):
            temp_crossover_loc = port_env.crossovers[yard_crane_assign_dict[yard_crane_idx][0].machine_list[6]].location
            getattr(sort_missions, "A_YARD_UA")(yard_crane_assign_dict[yard_crane_idx], temp_crossover_loc)
            port_env.yard_cranes[yard_crane_idx].location = [0, cf.SLOT_NUM_Y]
            for mission in yard_crane_assign_dict[yard_crane_idx]:
                assign_mission_to_yard(port_env.yard_cranes, mission, buffer_flag)


def assign_mission_to_yard(yard_cranes, mission, buffer_flag=False):
    if buffer_flag:
        mission.stage = 6
    else:
        mission.stage = 5
    curr_yard_loc = mission.yard_block_loc
    curr_yard_crane = yard_cranes['YC' + curr_yard_loc[0]]
    transfer_time_yard = mission.transfer_time_c2y
    moving_time_yard_crane = abs(
        curr_yard_crane.location[0] - curr_yard_loc[1]) * cf.SLOT_LENGTH / cf.YARDCRANE_SPEED_X + abs(
        cf.SLOT_NUM_Y - curr_yard_loc[2]) * cf.SLOT_WIDTH / cf.YARDCRANE_SPEED_Y * 2
    handling_time_yard_crane = mission.yard_crane_process_time
    arrive_time_yard = mission.total_process_time + mission.machine_start_time[0] + transfer_time_yard
    if any(curr_yard_crane.process_time):
        start_time_yard_crane = max(arrive_time_yard, curr_yard_crane.process_time[-1][-1])
    else:
        start_time_yard_crane = arrive_time_yard
    end_time_yard_crane = start_time_yard_crane + handling_time_yard_crane + moving_time_yard_crane
    # print("yard:" + mission.idx + " " + str(start_time_yard_crane - arrive_time_yard))
    curr_yard_crane.mission_list.append(mission)
    curr_yard_crane.process_time.append(
        [start_time_yard_crane, handling_time_yard_crane + moving_time_yard_crane, end_time_yard_crane])
    curr_yard_crane.location[0] = curr_yard_loc[1]
    curr_yard_crane.location[1] = cf.SLOT_NUM_Y
    mission.machine_list.append('a_yard')
    mission.machine_start_time.append(arrive_time_yard)
    mission.machine_process_time.append(start_time_yard_crane - arrive_time_yard)

    mission.machine_list.append(curr_yard_crane.idx)
    mission.machine_start_time.append(start_time_yard_crane)
    mission.machine_process_time.append(handling_time_yard_crane + moving_time_yard_crane)
    mission.total_process_time += transfer_time_yard + handling_time_yard_crane + moving_time_yard_crane \
                                  + start_time_yard_crane - arrive_time_yard
    mission.stage += 1


def reassign_mission_to_yard(lock_stations, crossovers, yard_cranes, mission, buffer_flag=False):
    curr_crossover = crossovers[mission.crossover_id]
    curr_crossover.mission_list.append(mission)
    temp_mission_list = curr_crossover.mission_list.clone()
    getattr(sort_missions, "A_CROSSOVER_NB")(temp_mission_list)
    del_machine(curr_crossover, buffer_flag)
    for mission in temp_mission_list:
        assign_mission_to_crossover(lock_stations, crossovers, mission, buffer_flag)
    for idx in curr_crossover.yard_block_list:
        del_machine(yard_cranes['YC' + idx], buffer_flag)
    for mission in temp_mission_list:
        assign_mission_to_yard(yard_cranes, mission, buffer_flag)


# ==================== operators  ====================
def process_inner_swap(machine, mission_A, mission_B, buffer_flag=False):
    i = machine.mission_list.index(mission_A)
    j = machine.mission_list.index(mission_B)
    machine.mission_list[i], machine.mission_list[j] = machine.mission_list[j], machine.mission_list[i]
    cur_mission_list = copy.copy(machine.mission_list)
    for mission in cur_mission_list:
        assign_mission_to_station(mission, machine, buffer_flag)
    machine.mission_list = cur_mission_list


def process_inner_relocate(machine, mission_a, pos, buffer_flag=False):
    cur_mission_list = copy.copy(machine.mission_list)
    cur_mission_list.remove(mission_a)
    cur_mission_list.insert(pos, mission_a)
    for mission in cur_mission_list:
        assign_mission_to_station(mission, machine, buffer_flag)
    machine.mission_list = cur_mission_list


def process_inner_relocate_yard(machine, mission_A, buffer_flag=False):
    cur_mission_list = copy.copy(machine.mission_list)
    cur_mission_list.remove(mission_A)
    list_len = len(cur_mission_list)
    pos = random.randint(0, list_len)
    cur_mission_list.insert(pos, mission_A)
    for mission in cur_mission_list:
        assign_mission_to_station(mission, machine, buffer_flag)
    machine.mission_list = cur_mission_list


def process_inter_swap(machine_A, machine_B, mission_A, mission_B, buffer_flag=False):
    i = machine_A.mission_list.index(mission_A)
    j = machine_B.mission_list.index(mission_B)
    machine_A.mission_list[i], machine_B.mission_list[j] = machine_B.mission_list[j], machine_A.mission_list[i]
    cur_A_mission_list = copy.copy(machine_A.mission_list)
    cur_B_mission_list = copy.copy(machine_B.mission_list)
    getattr(sort_missions, 'A_STATION')(cur_A_mission_list, machine_A)
    getattr(sort_missions, 'A_STATION')(cur_B_mission_list, machine_B)
    for mission in cur_A_mission_list:
        assign_mission_to_station(mission, machine_A, buffer_flag)
    machine_A.mission_list = cur_A_mission_list
    for mission in cur_B_mission_list:
        assign_mission_to_station(mission, machine_B, buffer_flag)
    machine_B.mission_list = cur_B_mission_list


def process_relocate_operator(mission, pos, machine_A, machine_B, buffer_flag=False):
    machine_A.mission_list.remove(mission)
    machine_B.mission_list.insert(pos + 1, mission)
    cur_A_mission_list = copy.copy(machine_A.mission_list)
    cur_B_mission_list = copy.copy(machine_B.mission_list)
    getattr(sort_missions, "A_STATION")(cur_A_mission_list, machine_A)
    getattr(sort_missions, "A_STATION")(cur_B_mission_list, machine_B)
    for mission in cur_A_mission_list:
        assign_mission_to_station(mission, machine_A, buffer_flag)
    machine_A.mission_list = cur_A_mission_list
    for mission in cur_B_mission_list:
        assign_mission_to_station(mission, machine_B, buffer_flag)
    machine_B.mission_list = cur_B_mission_list


def process_insert(mission, curr_station, buffer_flag=False):
    temp_mission_list = curr_station.mission_list.copy()
    temp_mission_list.append(mission)
    getattr(sort_missions, "A_STATION")(temp_mission_list, curr_station)
    for mission in temp_mission_list:
        assign_mission_to_station(mission, curr_station, buffer_flag)
    curr_station.mission_list = temp_mission_list


# ==================== get state (graph)  ====================
def get_station_graph_state(iter_solution):
    node_attr_list = []
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    for machine in iter_solution.lock_stations.values():
        for index in range(0, len(machine.mission_list) - 1):
            edge_in_list.append(int(machine.mission_list[index].idx[1:]) - 1)
            edge_out_list.append(int(machine.mission_list[index + 1].idx[1:]) - 1)
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    temp_mission_list = iter_solution.mission_list.copy()
    getattr(sort_missions, 'CHAR_ORDER')(temp_mission_list)
    for mission in temp_mission_list:
        if len(mission.machine_start_time) < 3:
            node_attr_list.append([0, 0, 0])
        else:
            node_attr_list.append(
                [mission.machine_start_time[2], mission.machine_process_time[4], mission.machine_process_time[2]])
    x = torch.tensor(node_attr_list).to(cf.DEVICE)
    data = Data(x=x, edge_index=edge_index)
    return data


def get_station_buffer_graph_state(iter_solution):
    node_attr_list = []
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    for machine in iter_solution.lock_stations.values():
        for buffer in machine.lock_station_buffers.values():
            for index in range(0, len(buffer.mission_list) - 1):
                edge_in_list.append(int(buffer.mission_list[index].idx[1:]) - 1)
                edge_out_list.append(int(buffer.mission_list[index + 1].idx[1:]) - 1)
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    temp_mission_list = iter_solution.mission_list.copy()
    getattr(sort_missions, 'CHAR_ORDER')(temp_mission_list)
    for mission in temp_mission_list:
        if len(mission.machine_start_time) < 3:
            node_attr_list.append([0, 0, 0])
        else:
            node_attr_list.append(
                [mission.machine_start_time[2], mission.machine_process_time[2] - mission.machine_process_time[3],
                 mission.machine_process_time[3]])
    x = torch.tensor(node_attr_list)
    data = Data(x=x, edge_index=edge_index)
    return data


def get_crossover_graph_state(iter_solution):
    node_attr_list = []
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    for machine in iter_solution.crossovers.values():
        for index in range(0, len(machine.mission_list) - 1):
            edge_in_list.append(int(machine.mission_list[index].idx[1:]) - 1)
            edge_out_list.append(int(machine.mission_list[index + 1].idx[1:]) - 1)
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    temp_mission_list = iter_solution.mission_list.copy()
    getattr(sort_missions, 'CHAR_ORDER')(temp_mission_list)
    for mission in temp_mission_list:
        if len(mission.machine_start_time) < 3:
            node_attr_list.append([0, 0, 0])
        else:
            node_attr_list.append(
                [mission.machine_start_time[5], mission.machine_process_time[6], mission.machine_process_time[5]])
    x = torch.tensor(node_attr_list)
    data = Data(x=x, edge_index=edge_index)
    return data


def get_yard_graph_state_bi_directed(iter_solution):
    adj_matrix = cal_yard_missions_matrix(iter_solution)
    node_attr_list = []
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    edge_attr_for_this_stage = []
    yard_crane_assign_dict = {}
    for yard_crane_idx in iter_solution.yard_cranes.keys():
        yard_crane_assign_dict.setdefault(yard_crane_idx, [])
    for machine in iter_solution.yard_cranes.values():
        if any(machine.mission_list):
            for i in range(0, len(machine.mission_list) - 1):
                edge_in_list.append(int(machine.mission_list[i].idx[1:]) - 1)
                edge_out_list.append(int(machine.mission_list[i + 1].idx[1:]) - 1)
                edge_attr_for_this_stage.append(
                    adj_matrix[int(machine.mission_list[i].idx[1:]) - 1][
                        int(machine.mission_list[i + 1].idx[1:]) - 1])
                for j in range(i + 2, len(machine.mission_list) - 1):
                    edge_in_list.append(int(machine.mission_list[i].idx[1:]) - 1)
                    edge_out_list.append(int(machine.mission_list[j].idx[1:]) - 1)
                    edge_in_list.append(int(machine.mission_list[j].idx[1:]) - 1)
                    edge_out_list.append(int(machine.mission_list[i].idx[1:]) - 1)
                    edge_attr_for_this_stage.append(adj_matrix[int(machine.mission_list[i].idx[1:]) - 1][
                                                        int(machine.mission_list[j].idx[1:]) - 1])
                    edge_attr_for_this_stage.append(adj_matrix[int(machine.mission_list[j].idx[1:]) - 1][
                                                        int(machine.mission_list[i].idx[1:]) - 1])
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    edge_attr = torch.tensor(edge_attr_for_this_stage).float()
    temp_mission_list = iter_solution.mission_list.copy()
    getattr(sort_missions, 'CHAR_ORDER')(temp_mission_list)
    for mission in temp_mission_list:
        if len(mission.machine_start_time) < 3:
            node_attr_list.append([0, 0, 0])
        else:
            node_attr_list.append(
                [mission.machine_start_time[7], mission.machine_process_time[8], mission.machine_process_time[7]])
    x = torch.tensor(node_attr_list)
    data = Data(x=x, edge_index=edge_index, edge_attr=edge_attr)
    return data


def get_yard_graph_state_with_attribute(iter_solution):
    adj_matrix = cal_yard_missions_matrix(iter_solution)
    node_attr_list = []
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    edge_attr_for_this_stage = []
    yard_crane_assign_dict = {}
    for yard_crane_idx in iter_solution.yard_cranes.keys():
        yard_crane_assign_dict.setdefault(yard_crane_idx, [])
    for machine in iter_solution.yard_cranes.values():
        if any(machine.mission_list):
            for i in range(0, len(machine.mission_list) - 1):
                edge_in_list.append(int(machine.mission_list[i].idx[1:]) - 1)
                edge_out_list.append(int(machine.mission_list[i + 1].idx[1:]) - 1)
                edge_attr_for_this_stage.append(
                    adj_matrix[int(machine.mission_list[i].idx[1:]) - 1][
                        int(machine.mission_list[i + 1].idx[1:]) - 1])
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    edge_attr = torch.tensor(edge_attr_for_this_stage).float()
    temp_mission_list = iter_solution.mission_list.copy()
    getattr(sort_missions, 'CHAR_ORDER')(temp_mission_list)
    for mission in temp_mission_list:
        if len(mission.machine_start_time) < 3:
            node_attr_list.append([0, 0, 0])
        else:
            node_attr_list.append(
                [mission.machine_start_time[7], mission.machine_process_time[8], mission.machine_process_time[7]])
    x = torch.tensor(node_attr_list)
    data = Data(x=x, edge_index=edge_index, edge_attr=edge_attr)
    return data


def get_yard_graph_state(iter_solution):
    node_attr_list = []
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    for machine in iter_solution.yard_cranes.values():
        for index in range(0, len(machine.mission_list) - 1):
            edge_in_list.append(int(machine.mission_list[index].idx[1:]) - 1)
            edge_out_list.append(int(machine.mission_list[index + 1].idx[1:]) - 1)
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    temp_mission_list = iter_solution.mission_list.copy()
    getattr(sort_missions, 'CHAR_ORDER')(temp_mission_list)
    for mission in temp_mission_list:
        if len(mission.machine_start_time) < 3:
            node_attr_list.append([0, 0, 0])
        else:
            node_attr_list.append(
                [mission.machine_start_time[7], mission.machine_process_time[8], mission.machine_process_time[7]])
    x = torch.tensor(node_attr_list)
    data = Data(x=x, edge_index=edge_index)
    return data


def get_current_release_mission_info(cur_mission):
    cur_mission_info = [cur_mission.transfer_time_e2s_min,
                        cur_mission.transfer_time_s2c_min,
                        cur_mission.transfer_time_c2y,
                        cur_mission.station_process_time,
                        cur_mission.yard_stop_loc[0],
                        cur_mission.yard_stop_loc[1]]
    return cur_mission_info


# ==================== get state (graph node - release mission)  ====================
def get_quay_release_state(qc_ls: dict, max_num: int):
    quay_cranes_attr = []
    seq_lengths = []
    for ls in qc_ls.values():
        quay_attr = []
        index = 0
        for i in range(len(ls)):
            mission = ls[i]
            if index == max_num:
                break
            else:
                bloc_idx = (ord(mission.yard_block_loc[0][0]) - ord('A')) * 4 + int(mission.yard_block_loc[0][-1])
                quay_attr.append(
                    [mission.machine_start_time[0], mission.machine_start_time[1], mission.station_process_time,
                     bloc_idx, int(mission.yard_block_loc[1]), int(mission.yard_block_loc[2])])
                index += 1
        for i in range(max_num - index):
            quay_attr.append([0, 0, 0, 0, 0, 0])
        quay_cranes_attr.append(quay_attr)
        seq_lengths.append(index)
    return quay_cranes_attr


def get_stations_release_state(ls_ls: dict, max_num: int):
    stations_attr = []
    seq_lengths = []
    for ls in ls_ls.values():
        getattr(sort_missions, "A_STATION_NB")(ls)
        station_attr = []
        index = 0
        for i in range(len(ls) - 1, -1, -1):
            mission = ls[i]
            if index == max_num:
                break
            else:
                bloc_idx = (ord(mission.yard_block_loc[0][0]) - ord('A')) * 4 + int(mission.yard_block_loc[0][-1])
                station_attr.append(
                    [mission.machine_start_time[2], mission.machine_start_time[4],
                     mission.machine_start_time[4] + mission.machine_process_time[4],
                     bloc_idx, int(mission.yard_block_loc[1]), int(mission.yard_block_loc[2])])
                index += 1
        for i in range(max_num - index):
            station_attr.append([0, 0, 0, 0, 0, 0])
        stations_attr.append(station_attr)
        seq_lengths.append(index)
    return stations_attr


def get_crossovers_release_state(co_ls: dict, max_num: int):
    crossovers_attr = []
    seq_lengths = []
    for ls in co_ls.values():
        getattr(sort_missions, "A_CROSSOVER_NB")(ls)
        crossover_attr = []
        index = 0
        for i in range(len(ls) - 1, -1, -1):
            mission = ls[i]
            if index == max_num:
                break
            else:
                bloc_idx = (ord(mission.yard_block_loc[0][0]) - ord('A')) * 4 + int(mission.yard_block_loc[0][-1])
                crossover_attr.append(
                    [mission.machine_start_time[5], mission.machine_start_time[6],
                     mission.machine_start_time[6] + mission.machine_process_time[6],
                     bloc_idx, int(mission.yard_block_loc[1]), int(mission.yard_block_loc[2])])
                index += 1
        for i in range(max_num - index):
            crossover_attr.append([0, 0, 0, 0, 0, 0])
        seq_lengths.append(index)
        crossovers_attr.append(crossover_attr)
    return crossovers_attr


def get_yards_release_state(yc_ls: dict, max_num: int):
    yards_attr = []
    seq_lengths = []
    for yard_crane_idx in yc_ls.keys():
        if yard_crane_idx in yc_ls:
            yard_attr = []
            index = 0
            ls = yc_ls[yard_crane_idx]
            getattr(sort_missions, "A_YARD_NB")(ls)
            for i in range(len(ls) - 1, -1, -1):
                mission = ls[i]
                if index == max_num:
                    continue
                else:
                    bloc_idx = (ord(mission.yard_block_loc[0][0]) - ord('A')) * 4 + int(mission.yard_block_loc[0][-1])
                    yard_attr.append(
                        [mission.machine_start_time[7], mission.machine_process_time[8],
                         mission.machine_start_time[8] + mission.machine_process_time[8],
                         bloc_idx, int(mission.yard_block_loc[1]), int(mission.yard_block_loc[2])])
                    index += 1
            for i in range(max_num - index):
                yard_attr.append([0, 0, 0, 0, 0, 0])
        else:
            yard_attr = []
            index = 0
            for i in range(max_num):
                yard_attr.append([0, 0, 0, 0, 0, 0])
        seq_lengths.append(index)
        yards_attr.append(yard_attr)
    return yards_attr


def get_stations_RNN_Sequence(iter_solution: PortEnv, cur_time: float, max_num: int = 1):
    stations_attr = []
    seq_lengths = []
    for station in iter_solution.lock_stations.values():
        station_attr = [[0, 0, 0]]
        index = 1
        for i in range(len(station.mission_list) - 1, -1, -1):
            mission = station.mission_list[i]
            if mission.machine_start_time[4] + mission.machine_process_time[4] <= cur_time or index == max_num + 1:
                continue
            else:
                station_attr.append(
                    [mission.machine_start_time[2], mission.machine_process_time[4], mission.machine_process_time[2]])
                index += 1
        for i in range(max_num - index + 1):
            station_attr.append([0, 0, 0])
        stations_attr.append(station_attr)
        seq_lengths.append(index)
    return stations_attr, seq_lengths


def get_crossovers_RNN_Sequence(iter_solution):
    crossovers_attr = []
    seq_lengths = []
    for crossover in iter_solution.crossovers.values():
        crossover_attr, seq_length = get_crossover_RNN_Sequence(crossover, 0)
        seq_lengths.append(seq_length)
        crossovers_attr.append(crossover_attr)
    return crossovers_attr, seq_lengths


def get_crossover_RNN_Sequence(crossover: Crossover, cur_time: float, max_num: int = 1):
    crossover_attr = [[0, 0, 0]]
    seq_length = 1
    for i in range(len(crossover.mission_list) - 1, -1, -1):
        mission = crossover.mission_list[i]
        if mission.machine_start_time[6] + mission.machine_process_time[6] <= cur_time or seq_length == max_num + 1:
            continue
        else:
            seq_length += 1
            crossover_attr.append(
                [mission.machine_start_time[5], mission.machine_process_time[6],
                 mission.machine_process_time[5]])
    for i in range(max_num - seq_length + 1):
        crossover_attr.append([0, 0, 0])
    return crossover_attr, seq_length


def get_yards_RNN_Sequence(iter_solution, yard_cranes_set):
    yards_attr = []
    seq_lengths = []
    for yard_crane_idx in yard_cranes_set:
        yard_crane = iter_solution.yard_cranes['YC' + yard_crane_idx]
        yard_attr, seq_length = get_yard_RNN_Sequence(yard_crane)
        seq_lengths.append(seq_length)
        yards_attr.append(yard_attr)
    return yards_attr, seq_lengths


def get_yard_RNN_Sequence(yard_crane: YardCrane, cur_time: float = 0, max_num: int = 1):
    yard_attr = [[0, 0, 0, 0, 0]]
    seq_length = 1
    for i in range(len(yard_crane.mission_list) - 1, -1, -1):
        mission = yard_crane.mission_list[i]
        if mission.machine_start_time[8] + mission.machine_process_time[8] <= cur_time or seq_length == max_num + 1:
            continue
        else:
            seq_length += 1
            yard_attr.append(
                [mission.machine_start_time[7], mission.machine_process_time[8],
                 mission.machine_process_time[7], mission.yard_block_loc[1], mission.yard_block_loc[2]])
    for i in range(max_num - seq_length + 1):
        yard_attr.append([0, 0, 0, 0, 0])
    return yard_attr, seq_length


# ==================== get state (New graph)  ====================
def get_rnn_state_v2(iter_solution: PortEnv, step_number: int, max_num: int, cur_mission: Mission = None):
    if cur_mission is None:
        cur_mission: Mission = iter_solution.mission_list[step_number]
    cur_mission_info = get_current_release_mission_info(cur_mission)
    if any(cur_mission.machine_start_time):
        cur_time = cur_mission.machine_start_time[1] + cur_mission.transfer_time_e2s_min
        stations_attr, stations_seq_lengths = get_stations_RNN_Sequence(iter_solution, cur_time, max_num)
        cur_time += cur_mission.station_process_time + cur_mission.transfer_time_s2c_min
        crossover_attr, crossover_seq_length = get_crossover_RNN_Sequence(
            iter_solution.crossovers[cur_mission.crossover_id], cur_time, max_num)
        cur_time += cur_mission.transfer_time_c2y
        yard_attr, yard_seq_length = get_yard_RNN_Sequence(
            iter_solution.yard_cranes['YC' + cur_mission.yard_block_loc[0]], cur_time, max_num)
        return cur_mission_info, stations_attr, crossover_attr, yard_attr
    else:
        return cur_mission_info, [[[0, 0, 0]], [[0, 0, 0]], [[0, 0, 0]], [[0, 0, 0]]], [[0, 0, 0]], [[0, 0, 0, 0, 0]]


def get_state_V1(self):
    data_list = [self.get_data_one_graph(self.lock_stations, 0), self.get_data_one_graph(self.crossovers, 1),
                 self.get_data_one_graph(self.yard_cranes, 2)]
    return data_list


def get_data_one_graph(self, machines, stage_index):
    node_attr_list = []
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    for machine in machines.values():
        for index in range(0, len(machine.mission_list) - 1):
            edge_in_list.append(int(machine.mission_list[index].idx[1:]) - 1)
            edge_out_list.append(int(machine.mission_list[index + 1].idx[1:]) - 1)
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    getattr(sort_missions, 'CHAR_ORDER')(self.mission_list)
    for mission in self.mission_list:
        node_attr_list.append([mission.waiting_time[stage_index], mission.process_time[stage_index],
                               mission.arriving_time[stage_index]])
    x = torch.tensor(node_attr_list)
    data = Data(x=x, edge_index=edge_index)
    return data


# ==================== get state (New graph)  ====================
def get_quay_cranes_state(iter_solution: PortEnv):
    node_attr_list = []
    for quay_crane in iter_solution.quay_cranes.values():
        node_attr_list.append(
            [quay_crane.time_to_exit, 0, 0])
    return node_attr_list


def get_stations_state(iter_solution: PortEnv, cur_time: float):
    node_attr_list = []
    for station in iter_solution.lock_stations.values():
        station_attr = [0, 0, 0]
        temp_cur_time = cur_time + iter_solution.exit_to_ls_matrix[int(station.idx[-1]) - 1] / (
                sum(cf.VEHICLE_SPEED) / 2.0)

        if any(station.mission_list):
            station_attr[0] = station.whole_occupy_time[-1][-1] - cur_time
            for i in range(len(station.mission_list) - 1, -1, -1):
                if station.process_time[i][-1] < temp_cur_time:
                    break
            station_attr[1] = len(station.mission_list) - i - 1
            if station.whole_occupy_time[-1][-1] > temp_cur_time:
                station_attr[2] = assign_mission_to_least_wait_station_buffer(station).wait_time_delay
        node_attr_list.append(station_attr)
    return node_attr_list


def get_crossovers_state(iter_solution: PortEnv, est_time: float, cur_time: float):
    node_attr_list = []
    for crossover in iter_solution.crossovers.values():
        crossover_attr = [0, 0, 0]
        if any(crossover.mission_list):
            crossover_attr[0] = crossover.process_time[-1][2] - cur_time
            for i in range(len(crossover.mission_list) - 1, -1, -1):
                if crossover.process_time[i][-1] <= est_time:
                    break
            handling_time_crossover = 5 * math.exp(len(crossover.mission_list) - i - 1) + 35
            if handling_time_crossover > 55:
                handling_time_crossover = 55
            crossover_attr[1] = len(crossover.mission_list) - i - 1
            crossover_attr[2] = handling_time_crossover
        node_attr_list.append(crossover_attr)
    return node_attr_list


def get_yard_crane_state(iter_solution: PortEnv, est_time: float, cur_mission: Mission, cur_time: float):
    node_attr_list = []
    for yard_crane in iter_solution.yard_cranes.values():
        yard_crane_attr = [0, 0, 300]
        temp_cur_time = est_time

        if yard_crane.idx[2:4] == cur_mission.yard_block_loc[0]:
            temp_cur_time = est_time + cur_mission.transfer_time_c2y

            yard_crane_attr[2] = abs(
                yard_crane.location[0] - cur_mission.yard_block_loc[1]) * cf.SLOT_LENGTH / yard_crane.x_move_rate + abs(
                yard_crane.location[1] - cur_mission.yard_block_loc[2]) * cf.SLOT_WIDTH / yard_crane.y_move_rate

        if any(yard_crane.mission_list):
            yard_crane_attr[0] = yard_crane.process_time[-1][2] - cur_time
            for i in range(len(yard_crane.mission_list) - 1, -1, -1):
                if yard_crane.process_time[i][-1] <= temp_cur_time:
                    break
                else:
                    yard_crane_attr[1] = len(yard_crane.mission_list) - i - 1

        node_attr_list.append(yard_crane_attr)
    return node_attr_list


def get_state(iter_solution: PortEnv, step_number: int = None, cur_mission: Mission = None):
    if cur_mission is None:
        cur_mission: Mission = iter_solution.mission_list[step_number]
    cur_time = cur_mission.machine_start_time[1]
    node_attr_list = []
    node_attr_list.extend(get_quay_cranes_state(iter_solution))
    node_attr_list.extend(get_stations_state(iter_solution, cur_time))
    tmp_time = get_est_arrive_crossover_time(iter_solution, cur_mission)
    node_attr_list.extend(get_crossovers_state(iter_solution, tmp_time, cur_time))
    node_attr_list.extend(get_yard_crane_state(iter_solution, tmp_time, cur_mission, cur_time))
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    edge_weight = []
    crossover_index, yard_crane_index = get_matched_crossover(cur_mission)
    for station in iter_solution.lock_stations.values():
        edge_in_list.append(int(cur_mission.quay_crane_id[-1]) - 1)
        edge_out_list.append(int(station.idx[-1]) - 1 + 3)
        edge_weight.append(iter_solution.exit_to_ls_matrix[int(station.idx[-1]) - 1] / cur_mission.vehicle_speed)
    for station in iter_solution.lock_stations.values():
        edge_in_list.append(int(station.idx[-1]) - 1 + 3)
        edge_out_list.append(crossover_index)
        edge_weight.append(iter_solution.ls_to_co_matrix[int(station.idx[-1]) - 1][
                               crossover_index - 8] / cur_mission.vehicle_speed)
    edge_in_list.append(crossover_index)
    edge_out_list.append(yard_crane_index)
    edge_weight.append(cur_mission.transfer_time_c2y)
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    edge_weight = torch.tensor(edge_weight, dtype=torch.float32) / 100
    x = torch.tensor(node_attr_list, dtype=torch.float32)
    norm = torch.tensor([[1 / 100, 0, 0], [0, 1 / 10, 0], [0, 0, 1 / 100]], dtype=torch.float32)
    x = torch.mm(x, norm)
    data = Data(x=x, edge_index=edge_index, edge_weight=edge_weight)
    return data


def get_state_n(env: PortEnv, step_number: int = None, cur_mission: Mission = None, max_num=5):
    if cur_mission is None:
        cur_mission: Mission = env.mission_list[step_number]
    cur_time = cur_mission.machine_start_time[1]
    qc_ls, qc_ls_ls, ls_ls, ls_co_ls, co_ls, co_yc_ls, yc_ls, f_ls = get_cur_time_status_v2(env, cur_time)
    node_attr_list = []
    node_attr_list.extend(get_quay_release_state(qc_ls, max_num))
    node_attr_list.extend(get_stations_release_state(ls_ls, max_num))
    node_attr_list.extend(get_crossovers_release_state(co_ls, max_num))
    node_attr_list.extend(get_yards_release_state(yc_ls, max_num))
    edge_in_list, edge_out_list = [], []
    edge_list_for_this_stage = []
    edge_weight = []
    qc_g_index = env.machine_name_to_idx[cur_mission.quay_crane_id]
    co_idx = get_matched_crossover(cur_mission)
    co_g_index = env.machine_name_to_idx[co_idx]
    yc_g_index = env.machine_name_to_idx['YC' + cur_mission.yard_block_loc[0]]
    for i in range(env.ls_num):
        station = env.lock_stations[list(env.lock_stations)[i]]
        ls_g_index = env.machine_name_to_idx[station.idx]
        edge_in_list.append(qc_g_index)
        edge_out_list.append(ls_g_index)
        edge_weight.append(env.exit_to_ls_matrix[i] / cur_mission.vehicle_speed)
    for i in range(env.ls_num):
        station = env.lock_stations[list(env.lock_stations)[i]]
        ls_g_index = env.machine_name_to_idx[station.idx]
        edge_in_list.append(ls_g_index)
        edge_out_list.append(co_g_index)
        edge_weight.append(env.ls_to_co_matrix[i][int(co_idx[-1]) - 1] / cur_mission.vehicle_speed)
    edge_in_list.append(co_g_index)
    edge_out_list.append(yc_g_index)
    edge_weight.append(cur_mission.transfer_time_c2y)
    edge_list_for_this_stage.append(edge_in_list)
    edge_list_for_this_stage.append(edge_out_list)
    edge_index = torch.tensor(edge_list_for_this_stage)
    edge_weight = torch.tensor(edge_weight, dtype=torch.float32) / 100
    x = torch.tensor(node_attr_list, dtype=torch.float32)
    norm = []
    for i in range(max_num):
        norm.extend([1 / 100, 1 / 100, 1 / 100, 1, 1, 1])
    norm = torch.tensor(norm)
    x = x.view(env.machine_num, -1) * norm
    data = Data(x=x, edge_index=edge_index, edge_weight=edge_weight)
    return data


def get_state_l(env: PortEnv, step_number: int = None, cur_mission: Mission = None, max_num=5):
    if cur_mission is None:
        cur_mission: Mission = env.mission_list[step_number]
    cur_time = cur_mission.machine_start_time[1]
    qc_ls, qc_ls_ls, ls_ls, ls_co_ls, co_ls, co_yc_ls, yc_ls, f_ls = get_cur_time_status_v2(env, cur_time)
    node_attr_list = []
    node_attr_list.extend(get_quay_release_state(qc_ls, max_num))
    node_attr_list.extend(get_stations_release_state(ls_ls, max_num))
    node_attr_list.extend(get_crossovers_release_state(co_ls, max_num))
    node_attr_list.extend(get_yards_release_state(yc_ls, max_num))
    x = torch.tensor(node_attr_list, dtype=torch.float32)
    norm = []
    for i in range(max_num):
        norm.extend([0, 0, 1 / 100, 0, 1, 0])
    norm = torch.tensor(norm)
    x = x.view(env.machine_num, -1) * norm
    return x
