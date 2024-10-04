#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：port_scheduling
@File    ：input_process.py
@Author  ：JacQ
@Date    ：2021/12/20 17:34
"""

import json
import os
import random
from copy import deepcopy
from statistics import mean

import numpy as np

import conf.configs as cf
from algorithm_factory.algo_utils.machine_cal_methods import match_mission_crossover, generate_instance_type, \
    generate_yard_blocks_set, split_integer
from common.buffer import Buffer
from common.crossover import Crossover
from common.iter_solution import IterSolution
from common.lock_station import LockStation
from common.lock_station_buffer import LockStationBuffer
from common.mission import Mission
from common.port_env import PortEnv
from common.quay_crane import QuayCrane
from common.yard_block import YardBlock
from common.yard_crane import YardCrane
from utils.log import Logger

logger = Logger().get_logger()

mission_count = 1
yard_blocks_set = ['A1', 'A2', 'A3', 'A4',
                   'B1', 'B2', 'B3', 'B4',
                   'C1', 'C2', 'C3', 'C4']


def read_json_from_file(file_name):
    with open(file_name, 'rb') as fd:
        data = fd.read()
    return json.loads(data.decode('utf-8'))


def write_json_to_file(file_name, data):
    with open(file_name, 'wb') as fd:
        fd.write(json.dumps(data, indent=4).encode('utf-8'))


# 生成堆场箱区
def generate_yard_blocks_info(block_to_location):
    yard_blocks_info = {}
    yard_blocks = {}
    for idx, location in block_to_location.items():
        yard_blocks_info[idx] = create_yard_block_dict(idx, location.tolist())
        yard_blocks[idx] = YardBlock(idx, location.tolist())
    return yard_blocks_info, yard_blocks


def create_yard_block_dict(idx, block_location):
    yard_block_info = {'idx': idx, 'block_location': block_location}
    return yard_block_info


# 生成场桥
def generate_yard_cranes_info(block_to_location):
    yard_cranes_info = {}
    yard_cranes = {}
    for idx, location in block_to_location.items():
        idx = 'YC' + idx
        handling_time = random.uniform(cf.YARDCRANE_HANDLING_TIME[0],
                                       cf.YARDCRANE_HANDLING_TIME[1])
        yard_cranes_info[idx] = create_yard_crane_dict(idx, idx, [0, cf.SLOT_NUM_Y],
                                                       cf.YARDCRANE_SPEED_X, cf.YARDCRANE_SPEED_Y,
                                                       handling_time
                                                       )
        yard_cranes[idx] = YardCrane(idx, idx, [0, 0],
                                     cf.YARDCRANE_SPEED_X, cf.YARDCRANE_SPEED_Y, handling_time)
    return yard_cranes_info, yard_cranes


def create_yard_crane_dict(idx, block_id, location, x_move_rate, y_move_rate, handling_time):
    yard_crane_info = {'idx': idx, 'block_id': block_id, 'location': location,
                       'x_move_rate': x_move_rate, 'y_move_rate': y_move_rate, 'handling_time': handling_time}
    return yard_crane_info


def generate_lock_station_buffers_info(lock_station_idx, station_location):
    lock_station_buffers_info = {}
    lock_station_buffers = {}
    for i in range(4):
        location = [station_location[0] - cf.LOCK_STATION_SPACE,
                    station_location[1] + (i + 1) * cf.LOCK_STATION_BUFFER_SPACE]
        wait_time_delay = cf.WAIT_TIME_DELAY[i]
        lock_station_buffers_info['L' + lock_station_idx + str(i + 1)] = create_lock_station_buffer_dict(
            'LS' + str(i + 1), location,
            lock_station_idx,
            wait_time_delay)
        lock_station_buffers['L' + lock_station_idx + str(i + 1)] = LockStationBuffer('LS' + str(i + 1), location,
                                                                                      lock_station_idx,
                                                                                      wait_time_delay)
    return lock_station_buffers_info, lock_station_buffers


def create_lock_station_buffer_dict(idx, location, lock_station_idx, wait_time_delay):
    lock_station_info = {'idx': idx, 'location': location, 'lock_station_idx': lock_station_idx,
                         'wait_time_delay': wait_time_delay}
    return lock_station_info


def generate_lock_stations_info(station_to_location):
    lock_stations_info = {}
    lock_stations = {}
    for idx, location in station_to_location.items():
        handling_time = random.uniform(cf.LOCK_STATION_HANDLING_TIME[0],
                                       cf.LOCK_STATION_HANDLING_TIME[1])
        lock_station_buffers_info, lock_station_buffers = generate_lock_station_buffers_info(idx, location.tolist())
        lock_stations_info[idx] = create_lock_station_dict(idx, location.tolist(), 5,
                                                           handling_time, lock_station_buffers_info)
        lock_stations[idx] = LockStation(idx, location.tolist(), 5, lock_station_buffers)
    return lock_stations_info, lock_stations


def create_lock_station_dict(idx, location, capacity, handling_time, lock_station_buffers):
    lock_station_info = {'idx': idx, 'location': location, 'capacity': capacity,
                         'handling_time': handling_time, 'lock_station_buffers': lock_station_buffers}
    return lock_station_info


def generate_missions_info(idx, cur_yar_blocks, m_num):
    missions_info = {}
    missions = {}
    global mission_count

    for m in range(m_num):
        vehicle_speed = (cf.VEHICLE_SPEED[0] + cf.VEHICLE_SPEED[1]) / 2
        station_process_time = random.uniform(cf.LOCK_STATION_HANDLING_TIME[0], cf.LOCK_STATION_HANDLING_TIME[1])
        intersection_process_time = random.uniform(cf.CROSSOVER_HANDLING_TIME[0], cf.CROSSOVER_HANDLING_TIME[1])
        yard_crane_process_time = random.uniform(cf.YARDCRANE_HANDLING_TIME[0], cf.YARDCRANE_HANDLING_TIME[1])
        yard_block_loc = (random.choice(cur_yar_blocks), random.randint(0, cf.SLOT_NUM_X - 1),
                          random.randint(0, cf.SLOT_NUM_Y - 1))
        missions_info['M' + str(mission_count)] = create_mission_dict('M' + str(mission_count), idx,
                                                                      yard_block_loc,
                                                                      yard_crane_process_time,
                                                                      True, 0 + cf.RELEASE_TIME * m,
                                                                      vehicle_speed, station_process_time,
                                                                      intersection_process_time)
        missions['M' + str(mission_count)] = Mission('M' + str(mission_count), idx, yard_block_loc,
                                                     yard_crane_process_time,
                                                     True, 0 + cf.RELEASE_TIME * m, vehicle_speed,
                                                     station_process_time, intersection_process_time)
        mission_count += 1
    return missions_info, missions


def create_mission_dict(idx, quay_crane_id, yard_block_loc, yard_crane_process_time, locked,
                        release_time, vehicle_speed, station_process_time, intersection_process_time):
    mission_info = {'idx': idx, 'quay_crane_id': quay_crane_id, 'yard_block_loc': yard_block_loc,
                    'yard_crane_process_time': yard_crane_process_time, 'locked': locked,
                    'release_time': release_time, 'vehicle_speed': vehicle_speed,
                    'station_process_time': station_process_time,
                    'intersection_process_time': intersection_process_time}
    return mission_info


def generate_quay_cranes_info(quay_crane_to_location, qc_num, is_num, yc_num, m_num):
    quay_cranes_info = {}
    quay_cranes = {}
    cur_yar_blocks = generate_yard_blocks_set(is_num, yc_num)
    cnt = 0
    m_num_ls = split_integer(m_num, qc_num)
    for idx, location in quay_crane_to_location.items():
        if cnt >= qc_num:
            break
        missions_info, missions = generate_missions_info(idx, cur_yar_blocks, m_num_ls[cnt])
        quay_cranes_info[idx] = create_quay_crane_dict(idx, missions_info, location.tolist())
        quay_cranes[idx] = QuayCrane(idx, missions, location.tolist())
        cnt = cnt + 1
    return quay_cranes_info, quay_cranes


def generate_quay_cranes_info_revised_qc(quay_crane_to_location, qc_num, is_num, yc_num, m_num, all_missions):
    quay_cranes_info = {}
    cnt, cntt = 0, 0
    m_num_ls = split_integer(m_num, qc_num)
    for idx, location in quay_crane_to_location.items():
        if cnt >= qc_num:
            break
        missions_info = {}
        for idxx in list(all_missions)[cntt:m_num_ls[cnt] + cntt]:
            all_missions[idxx]['quay_crane_id'] = idx
            missions_info[idxx] = all_missions[idxx]
        cntt += m_num_ls[cnt]
        quay_cranes_info[idx] = create_quay_crane_dict(idx, missions_info, location.tolist())
        cnt = cnt + 1
    return quay_cranes_info


def generate_quay_cranes_info_revised_yc(quay_crane_to_location, qc_num, is_num, yc_num, m_num, all_missions):
    quay_cranes_info = {}
    cnt, cntt = 0, 0
    m_num_ls = split_integer(m_num, qc_num)
    cur_yar_blocks = generate_yard_blocks_set(is_num, yc_num)
    for idx, location in quay_crane_to_location.items():
        if cnt >= qc_num:
            break
        missions_info = {}
        for idxx in list(all_missions)[cntt:m_num_ls[cnt] + cntt]:
            all_missions[idxx]['yard_block_loc'] = (
                random.choice(cur_yar_blocks), all_missions[idxx]['yard_block_loc'][1],
                all_missions[idxx]['yard_block_loc'][2])
            missions_info[idxx] = all_missions[idxx]
        cntt += m_num_ls[cnt]
        quay_cranes_info[idx] = create_quay_crane_dict(idx, missions_info, location.tolist())
        cnt = cnt + 1
    return quay_cranes_info


def create_quay_crane_dict(idx, missions, location):
    quay_crane_info = {'idx': idx, 'missions': missions, 'location': location}
    return quay_crane_info


def generate_buffers_info(quay_crane_to_location):
    buffers_info = {}
    buffers = {}
    i = 1
    for idx, location in quay_crane_to_location.items():
        buffers_info['BF' + str(i)] = create_buffer_dict('BF' + str(i), location.tolist(), cf.BUFFER_PROCESS_TIME, 5)
        buffers['BF' + str(i)] = Buffer('BF' + str(i), location.tolist(), cf.BUFFER_PROCESS_TIME, 5)
        i += 1
    return buffers_info, buffers


def create_buffer_dict(idx, location, handling_time, capacity):
    buffer_info = {'idx': idx, 'location': location, 'handling_time': handling_time, 'capacity': capacity}
    return buffer_info


def generate_crossovers_info(crossovers_to_location, yard_blocks_to_crossover):
    crossovers_info = {}
    crossovers = {}
    for idx, location in crossovers_to_location.items():
        yard_block_list = yard_blocks_to_crossover[idx]
        crossovers_info[idx] = create_crossover_dict(idx, location.tolist(), yard_block_list)
        crossovers[idx] = Crossover(idx, location.tolist(), yard_block_list)
    return crossovers_info, crossovers


def create_crossover_dict(idx, location, yard_block_list):
    crossover_info = {'idx': idx, 'location': location, 'yard_block_list': yard_block_list}
    return crossover_info


def cal_block_to_location():
    first_block = cf.A1_LOCATION
    b_dx = cf.BLOCK_SPACE_X
    b_dy = cf.BLOCK_SPACE_Y
    block_to_location = {'A1': first_block, 'A2': first_block + np.array([0, b_dy]),
                         'A3': first_block + np.array([0, 2 * b_dy]),
                         'A4': first_block + np.array([0, 3 * b_dy]),
                         'B1': first_block + np.array([b_dx, 0]), 'B2': first_block + np.array([b_dx, b_dy]),
                         'B3': first_block + np.array([b_dx, 2 * b_dy]),
                         'B4': first_block + np.array([b_dx, 3 * b_dy]),
                         'C1': first_block + np.array([2 * b_dx, 0]), 'C2': first_block + np.array([2 * b_dx, b_dy]),
                         'C3': first_block + np.array([2 * b_dx, 2 * b_dy]),
                         'C4': first_block + np.array([2 * b_dx, 3 * b_dy])}
    return block_to_location


def cal_quay_crane_to_location(qc_num):
    first_quay_crane = cf.QUAY_EXIT + (cf.QUAYCRANE_EXIT, 0)
    qc_dx = cf.QUAYCRANE_CRANE
    quay_crane_to_location = {}
    for m in range(qc_num):
        quay_crane_to_location['QC' + str(m + 1)] = first_quay_crane + np.array([m * qc_dx, 0])
    return quay_crane_to_location


def cal_station_to_location(ls_num):
    first_station = cf.S1_STATION_LOCATION
    s_dx = cf.LOCK_STATION_SPACE
    station_to_location = {}
    for m in range(ls_num):
        station_to_location['S' + str(m + 1)] = first_station + np.array([m * s_dx, 0])
    return station_to_location


def cal_yard_blocks_to_crossover(is_num, block_to_location):
    crossovers_to_location, yard_blocks_to_crossover = {}, {}
    yard_blocks_to_crossover_t = [['A1', 'A2', 'A3', 'A4'], ['B1', 'B2', 'B3', 'B4'], ['C1', 'C2', 'C3', 'C4']]
    for i in range(is_num):
        crossovers_to_location['CO' + str(i + 1)] = block_to_location[yard_blocks_to_crossover_t[i][0]] + \
                                                    np.array([cf.BLOCK_IS_SPACE_X, -cf.BLOCK_IS_SPACE_Y]) + np.array(
            [cf.SLOT_LENGTH * cf.SLOT_NUM_X, -cf.SLOT_WIDTH * cf.SLOT_NUM_Y])
        yard_blocks_to_crossover['CO' + str(i + 1)] = yard_blocks_to_crossover_t[i]
    return crossovers_to_location, yard_blocks_to_crossover


def missions_dict_to_obj(machine, flag):
    if hasattr(machine, 'missions') and flag == 1:
        missions = {}
        for idx, mission in machine.missions.items():
            cur_mission = object.__new__(Mission)
            cur_mission.__dict__ = mission
            cur_mission.total_process_time = 0
            cur_mission.machine_list = []
            cur_mission.machine_start_time = []
            cur_mission.machine_process_time = []
            cur_mission.stage = 1
            missions[idx] = cur_mission
        return missions
    if hasattr(machine, 'mission_list') and flag == 2:
        mission_list = []
        for idx, mission in machine.mission_list.items():
            cur_mission = object.__new__(Mission)
            cur_mission.__dict__ = mission
            cur_mission.total_process_time = 0
            cur_mission.machine_list = []
            cur_mission.machine_start_time = []
            cur_mission.machine_process_time = []
            cur_mission.stage = 1
            mission_list.append(cur_mission)
        return mission_list


def generate_data_for_test(inst_idx, inst_type='C', mission_num=-1):
    global mission_count
    mission_count = 1
    qc_num, ls_num, is_num, yc_num, m_num = generate_instance_type(inst_type, mission_num)
    quay_crane_to_location = cal_quay_crane_to_location(qc_num)
    quay_cranes_info, quay_cranes = generate_quay_cranes_info(quay_crane_to_location, qc_num, is_num, yc_num, m_num)
    buffers_info, buffers = generate_buffers_info(quay_crane_to_location)
    station_to_location = cal_station_to_location(ls_num)
    lock_stations_info, lock_stations = generate_lock_stations_info(station_to_location)
    block_to_location = cal_block_to_location()
    yard_blocks_info, yard_blocks = generate_yard_blocks_info(block_to_location)
    yard_cranes_info, yard_cranes = generate_yard_cranes_info(block_to_location)
    crossovers_to_location, yard_blocks_to_crossover = cal_yard_blocks_to_crossover(is_num, block_to_location)
    crossovers_info, crossovers = generate_crossovers_info(crossovers_to_location, yard_blocks_to_crossover)
    input_data = {'quay_cranes': list(quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'train_' + inst_type + '_' + str(inst_idx) + '.json'), input_data)
    # write_json_to_file(os.path.join(cf.DATA_PATH, str(inst_idx) + '.json'), input_data)
    instance = PortEnv(quay_cranes, buffers, lock_stations, crossovers, yard_blocks, yard_cranes,
                       (qc_num, ls_num, is_num, yc_num, m_num))
    return instance


def generate_data_for_another_profile(inst_idx, inst_type, mission_num=10):
    global mission_count
    mission_count = 1
    qc_num, ls_num, is_num, yc_num, m_num = generate_instance_type(inst_type, mission_num)
    quay_crane_to_location = cal_quay_crane_to_location(qc_num)
    quay_cranes_info, quay_cranes = generate_quay_cranes_info(quay_crane_to_location, qc_num, is_num, yc_num, m_num)
    all_missions = {}
    for qc in quay_cranes_info.values():
        all_missions.update(qc['missions'])
    buffers_info, buffers = generate_buffers_info(quay_crane_to_location)
    station_to_location = cal_station_to_location(ls_num)
    lock_stations_info, lock_stations = generate_lock_stations_info(station_to_location)
    block_to_location = cal_block_to_location()
    yard_blocks_info, yard_blocks = generate_yard_blocks_info(block_to_location)
    yard_cranes_info, yard_cranes = generate_yard_cranes_info(block_to_location)
    crossovers_to_location, yard_blocks_to_crossover = cal_yard_blocks_to_crossover(is_num, block_to_location)
    crossovers_info, crossovers = generate_crossovers_info(crossovers_to_location, yard_blocks_to_crossover)
    input_data = {'quay_cranes': list(quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'C/' + str(inst_idx) + '.json'), input_data)

    # 'QC-1'
    tmp_all_missions = deepcopy(all_missions)
    new_quay_crane_to_location = cal_quay_crane_to_location(qc_num - 1)
    new_quay_cranes_info = generate_quay_cranes_info_revised_qc(new_quay_crane_to_location, qc_num - 1, is_num,
                                                                yc_num, m_num, tmp_all_missions)
    new_buffers_info, _ = generate_buffers_info(new_quay_crane_to_location)
    input_data = {'quay_cranes': list(new_quay_cranes_info.values()), 'buffers': list(new_buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'QC-1/' + str(inst_idx) + '.json'), input_data)
    # 'QC+1'
    tmp_all_missions = deepcopy(all_missions)
    new_quay_crane_to_location = cal_quay_crane_to_location(qc_num + 1)
    new_quay_cranes_info = generate_quay_cranes_info_revised_qc(new_quay_crane_to_location, qc_num + 1, is_num,
                                                                yc_num, m_num, tmp_all_missions)
    new_buffers_info, _ = generate_buffers_info(new_quay_crane_to_location)
    input_data = {'quay_cranes': list(new_quay_cranes_info.values()), 'buffers': list(new_buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'QC+1/' + str(inst_idx) + '.json'), input_data)
    # 'YC-1'
    tmp_all_missions = deepcopy(all_missions)
    new_quay_cranes_info = generate_quay_cranes_info_revised_yc(quay_crane_to_location, qc_num, is_num,
                                                                yc_num - 1, m_num, tmp_all_missions)
    input_data = {'quay_cranes': list(new_quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    yard_cranes_set = []
    for quay_crane in new_quay_cranes_info.values():
        for mission in quay_crane['missions'].values():
            if mission['yard_block_loc'][0] not in yard_cranes_set:
                yard_cranes_set.append(mission['yard_block_loc'][0])
    if len(yard_cranes_set) < yc_num - 1:
        print(inst_idx)
        print(yard_cranes_set)
    write_json_to_file(os.path.join(cf.DATA_PATH, 'YC-1/' + str(inst_idx) + '.json'), input_data)
    # 'YC+1'
    tmp_all_missions = deepcopy(all_missions)
    new_quay_cranes_info = generate_quay_cranes_info_revised_yc(quay_crane_to_location, qc_num, is_num,
                                                                yc_num + 1, m_num, tmp_all_missions)
    input_data = {'quay_cranes': list(new_quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'YC+1/' + str(inst_idx) + '.json'), input_data)
    # 'IS-1'
    tmp_all_missions = deepcopy(all_missions)
    new_quay_cranes_info = generate_quay_cranes_info_revised_yc(quay_crane_to_location, qc_num, is_num - 1,
                                                                yc_num, m_num, tmp_all_missions)
    new_crossovers_to_location, new_yard_blocks_to_crossover = cal_yard_blocks_to_crossover(is_num - 1,
                                                                                            block_to_location)
    new_crossovers_info, _ = generate_crossovers_info(new_crossovers_to_location, new_yard_blocks_to_crossover)
    input_data = {'quay_cranes': list(new_quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(new_crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'IS-1/' + str(inst_idx) + '.json'), input_data)
    # 'IS+1'
    tmp_all_missions = deepcopy(all_missions)
    new_quay_cranes_info = generate_quay_cranes_info_revised_yc(quay_crane_to_location, qc_num, is_num + 1,
                                                                yc_num, m_num, tmp_all_missions)
    new_crossovers_to_location, new_yard_blocks_to_crossover = cal_yard_blocks_to_crossover(is_num + 1,
                                                                                            block_to_location)
    new_crossovers_info, _ = generate_crossovers_info(new_crossovers_to_location, new_yard_blocks_to_crossover)
    input_data = {'quay_cranes': list(new_quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(lock_stations_info.values()), 'crossovers': list(new_crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'IS+1/' + str(inst_idx) + '.json'), input_data)
    # 'LS-1'
    station_to_location = cal_station_to_location(ls_num - 1)
    new_lock_stations_info, lock_stations = generate_lock_stations_info(station_to_location)
    input_data = {'quay_cranes': list(quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(new_lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'LS-1/' + str(inst_idx) + '.json'), input_data)
    # 'LS+1'
    station_to_location = cal_station_to_location(ls_num + 1)
    new_lock_stations_info, lock_stations = generate_lock_stations_info(station_to_location)
    input_data = {'quay_cranes': list(quay_cranes_info.values()), 'buffers': list(buffers_info.values()),
                  'lock_stations': list(new_lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                  'yard_blocks': list(yard_blocks_info.values()), 'yard_cranes': list(yard_cranes_info.values())}
    write_json_to_file(os.path.join(cf.DATA_PATH, 'LS+1/' + str(inst_idx) + '.json'), input_data)


def write_env_to_file(env: PortEnv, train_num, mission_num_one_crane):
    lock_station_dict = {}
    for lock_station in env.lock_stations.values():
        l_dict = {'idx': lock_station.idx, 'mission_list': list(lock_station.mission_list)}
        lock_station_dict.setdefault(str(lock_station.idx), list(l_dict))

    input_data = {'mission_list': env.mission_list, 'quay_cranes': list(env.quay_cranes.values()),
                  'buffers': list(env.buffers.values()),
                  'lock_stations': list(env.lock_stations.values()), 'crossovers': list(env.crossovers.values()),
                  'yard_blocks': list(env.yard_blocks.values()), 'yard_cranes': list(env.yard_cranes.values())}
    write_json_to_file(
        os.path.join(cf.DATA_PATH, 'solu_' + str(train_num) + '_' + str(mission_num_one_crane) + '.json'), input_data)


def read_env_to_file(train_num, mission_num_one_crane):
    filepath = os.path.join(cf.DATA_PATH, 'train_' + str(train_num) + '_' + str(mission_num_one_crane) + '.json')
    input_data = read_json_from_file(filepath)
    quay_cranes = read_quay_cranes_info(input_data.get('quay_cranes'))
    buffers = read_buffers_info(input_data.get('buffers'))
    lock_stations = read_lock_stations_info(input_data.get('lock_stations'))
    crossovers = read_crossovers_info(input_data.get('crossovers'))
    yard_blocks = read_yard_blocks_info(input_data.get('yard_blocks'))
    yard_cranes = read_yard_cranes_info(input_data.get('yard_cranes'))
    port_env = PortEnv(quay_cranes, buffers, lock_stations, crossovers, yard_blocks, yard_cranes)
    mission_list = read_yard_cranes_info(input_data.get('mission_list'))
    port_env.mission_list = mission_list
    cal_transfer_time(port_env)
    iter_solution = IterSolution(port_env)
    return iter_solution


def read_yard_blocks_info(yard_blocks_info):
    yard_blocks = {}
    for yard_block_info in yard_blocks_info:
        yard_block = object.__new__(YardBlock)
        yard_block.__dict__ = yard_block_info
        if int(yard_block.idx[1:]) % 2 == 1:
            lane_loc = yard_block.block_location + np.array([0, cf.LANE_WIDTH])
        else:
            lane_loc = yard_block.block_location + np.array([0, -cf.LANE_WIDTH])
        lane_loc = lane_loc.tolist()
        yard_block.slot_width = cf.SLOT_WIDTH
        yard_block.slot_length = cf.SLOT_LENGTH
        yard_block.lane_loc = lane_loc
        yard_blocks[yard_block.idx] = yard_block
    return yard_blocks


def read_yard_cranes_info(yard_cranes_info):
    yard_cranes = {}
    for yard_crane_info in yard_cranes_info:
        yard_crane = object.__new__(YardCrane)
        yard_crane.__dict__ = yard_crane_info
        if not hasattr(yard_crane, 'mission_list'):
            yard_crane.mission_list = []
            yard_crane.process_time = []
        else:
            yard_crane.mission_list = missions_dict_to_obj(yard_crane, 2)
        yard_cranes[yard_crane.idx] = yard_crane
    return yard_cranes


def read_lock_stations_info(lock_stations_info):
    lock_stations = {}
    for lock_station_info in lock_stations_info:
        lock_station = object.__new__(LockStation)
        lock_station.__dict__ = lock_station_info
        lock_station.lock_station_buffers = station_buffer_dict_to_obj(lock_station)
        if not hasattr(lock_station, 'mission_list'):
            lock_station.mission_list = []
            lock_station.process_time = []
            lock_station.whole_occupy_time = []
        else:
            lock_station.mission_list = missions_dict_to_obj(lock_station, 2)
        lock_stations[lock_station.idx] = lock_station
    return lock_stations


def station_buffer_dict_to_obj(lock_station):
    lock_station_buffers = {}
    for idx, station_buffer in lock_station.lock_station_buffers.items():
        cur_buffer = object.__new__(LockStationBuffer)
        cur_buffer.__dict__ = station_buffer
        if not hasattr(cur_buffer, 'mission_list'):
            cur_buffer.mission_list = []
            cur_buffer.process_time = []
        else:
            cur_buffer.mission_list = missions_dict_to_obj(cur_buffer, 2)
        lock_station_buffers[idx] = cur_buffer
    return lock_station_buffers


def read_missions_info(missions_info):
    missions = {}
    for idx, mission_info in missions_info.items():
        mission = object.__new__(Mission)
        mission.__dict__ = mission_info
        mission.total_process_time = 0
        mission.stage = 1
        if not hasattr(mission, 'machine_list'):
            mission.machine_list = []
            mission.machine_start_time = []
            mission.machine_process_time = []
        missions[mission.idx] = mission
    return missions


def read_quay_cranes_info(quay_cranes_info):
    quay_cranes = {}
    for quay_crane_info in quay_cranes_info:
        quay_crane = object.__new__(QuayCrane)
        quay_crane.__dict__ = quay_crane_info
        quay_crane.missions = missions_dict_to_obj(quay_crane, 1)
        if not hasattr(quay_crane, 'mission_list'):
            quay_crane.mission_list = []
            quay_crane.process_time = []
        else:
            quay_crane.mission_list = missions_dict_to_obj(quay_crane, 2)
        quay_cranes[quay_crane.idx] = quay_crane
    return quay_cranes


def read_buffers_info(buffers_info):
    buffers = {}
    for buffer_info in buffers_info:
        buffer = object.__new__(Buffer)
        buffer.__dict__ = buffer_info
        if not hasattr(buffer, 'mission_list'):
            buffer.mission_list = []
            buffer.process_time = []
        else:
            buffer.mission_list = missions_dict_to_obj(buffer, 2)
        buffers[buffer.idx] = buffer
    return buffers


def read_crossovers_info(crossovers_info):
    crossovers = {}
    for crossover_info in crossovers_info:
        crossover = object.__new__(Crossover)
        crossover.__dict__ = crossover_info
        if not hasattr(crossover, 'mission_list'):
            crossover.mission_list = []
            crossover.process_time = []
        else:
            crossover.mission_list = missions_dict_to_obj(crossover, 2)
        crossovers[crossover.idx] = crossover
    return crossovers


def create_cur_missions_info(machine):
    cur_missions_info = {}
    for mission in machine.mission_list:
        cur_missions_info[mission.idx] = mission.__dict__
    return cur_missions_info


def cal_transfer_time(instance: PortEnv):
    for station in instance.lock_stations.values():
        station.distance_to_exit = (abs(station.location[0] - cf.QUAY_EXIT[0]) + abs(
            station.location[1] - cf.QUAY_EXIT[1]))
        instance.exit_to_ls_matrix[int(station.idx[-1]) - 1] = station.distance_to_exit
        for crossover in instance.crossovers.values():
            instance.ls_to_co_matrix[int(station.idx[-1]) - 1][int(crossover.idx[-1]) - 1] = abs(
                crossover.location[0] - station.location[0]) + abs(
                crossover.location[1] - station.location[1])
    instance.exit_to_station_min = min(instance.exit_to_ls_matrix)
    instance.ls_to_co_min = [min(row) for row in
                             list(map(list, zip(*instance.ls_to_co_matrix)))]
    for quay_crane in instance.quay_cranes.values():
        quay_crane.time_to_exit = (abs(quay_crane.location[0] - cf.QUAY_EXIT[0]) + abs(
            quay_crane.location[1] - cf.QUAY_EXIT[1])) / (sum(cf.VEHICLE_SPEED) / 2.0)
        for mission in quay_crane.missions.values():
            curr_yard_loc = mission.yard_block_loc
            curr_yard_block = instance.yard_blocks[curr_yard_loc[0]]
            mission.yard_stop_loc = [curr_yard_loc[1] * cf.SLOT_LENGTH + curr_yard_block.block_location[0],
                                     curr_yard_block.lane_loc[1]]
            crossover = match_mission_crossover(instance.crossovers, mission)
            mission.crossover_id = crossover.idx
            crossover_loc = crossover.location
            mission.transfer_time_e2s_min = instance.exit_to_station_min / mission.vehicle_speed
            mission.transfer_time_s2c_min = instance.ls_to_co_min[
                                                int(crossover.idx[-1]) - 1] / mission.vehicle_speed
            mission.transfer_time_c2y = (abs(mission.yard_stop_loc[0] - crossover_loc[0]) + abs(
                mission.yard_stop_loc[1] - crossover_loc[1])) / mission.vehicle_speed


def read_input(pre, inst_idx, inst_type, mission_num=None) -> IterSolution:
    if pre == 'profile':
        filepath = os.path.join(cf.ROOT_FOLDER_PATH,
                                'data/data_profile/' + inst_type + '/' + str(inst_idx) + '.json')
    if pre == 'workload':
        filepath = os.path.join(cf.ROOT_FOLDER_PATH, 'data/profile/' + inst_type + '.json')
    else:
        filepath = os.path.join(cf.ROOT_FOLDER_PATH,
                                'data/data_' + inst_type + '/' + pre + '_' + inst_type + '_' + str(inst_idx) + '.json')
    input_data = read_json_from_file(filepath)
    quay_cranes = read_quay_cranes_info(input_data.get('quay_cranes'))
    buffers = read_buffers_info(input_data.get('buffers'))
    lock_stations = read_lock_stations_info(input_data.get('lock_stations'))
    crossovers = read_crossovers_info(input_data.get('crossovers'))
    yard_blocks = read_yard_blocks_info(input_data.get('yard_blocks'))
    yard_cranes = read_yard_cranes_info(input_data.get('yard_cranes'))
    cell = generate_instance_type(inst_type)
    if mission_num is not None:
        cell = (cell[0], cell[1], cell[2], cell[3], mission_num)
    port_env = PortEnv(quay_cranes, buffers, lock_stations, crossovers, yard_blocks, yard_cranes, cell)
    cal_transfer_time(port_env)
    iter_solution = IterSolution(port_env)
    return iter_solution


def output_result(port_env):
    quay_cranes_info = {}
    buffers_info = {}
    lock_stations_info = {}
    crossovers_info = {}
    yard_blocks_info = {}
    yard_cranes_info = {}
    missions_info = {}
    for mission in port_env.mission_list:
        missions_info[mission.idx] = mission.__dict__
    for idx, machine in port_env.quay_cranes.items():
        quay_cranes_info[machine.idx] = machine.__dict__
        quay_cranes_info[machine.idx]['missions'] = create_cur_missions_info(machine)
        quay_cranes_info[machine.idx]['mission_list'] = create_cur_missions_info(machine)
    for idx, machine in port_env.buffers.items():
        buffers_info[machine.idx] = machine.__dict__
        buffers_info[machine.idx]['mission_list'] = create_cur_missions_info(machine)
    for idx, machine in port_env.lock_stations.items():
        lock_stations_info[machine.idx] = machine.__dict__
        lock_stations_info[machine.idx]['mission_list'] = create_cur_missions_info(machine)
    for idx, machine in port_env.crossovers.items():
        crossovers_info[machine.idx] = machine.__dict__
        crossovers_info[machine.idx]['mission_list'] = create_cur_missions_info(machine)
    for idx, machine in port_env.yard_blocks.items():
        yard_blocks_info[machine.idx] = machine.__dict__
    for idx, machine in port_env.yard_cranes.items():
        yard_cranes_info[machine.idx] = machine.__dict__
        yard_cranes_info[machine.idx]['mission_list'] = create_cur_missions_info(machine)
    output_data = {'quay_cranes': list(quay_cranes_info.values()),
                   'buffers': list(buffers_info.values()),
                   'lock_stations': list(lock_stations_info.values()), 'crossovers': list(crossovers_info.values()),
                   'yard_cranes': list(yard_cranes_info.values()), 'missions': list(missions_info.values()),
                   'yard_blocks': list(yard_blocks_info.values())}
    write_json_to_file(cf.OUTPUT_PATH, output_data)


def count_yard_block_assign(port_env):
    yard_block_count = {}
    for quay_crane in port_env.quay_cranes.values():
        for mission in quay_crane.missions.values():
            num = yard_block_count.setdefault(mission.yard_block_loc[0], 0)
            yard_block_count[mission.yard_block_loc[0]] = num + 1

    for idx, num in sorted(yard_block_count.items()):
        print(idx + ": " + str(num), end='\t')
