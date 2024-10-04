#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：Port_env.py
@Author  ：JacQ
@Date    ：2022/1/4 16:17
"""

from utils.log import Logger

logger = Logger(name='root').get_logger()


class PortEnv:
    def __init__(self, quay_cranes, buffers, lock_stations, crossovers, yard_blocks, yard_cranes, inst_type):
        self.quay_cranes = quay_cranes
        self.buffers = buffers
        self.lock_stations = lock_stations
        self.crossovers = crossovers
        self.yard_blocks = yard_blocks
        self.yard_cranes = yard_cranes
        self.yard_cranes_set = self.get_yard_cranes_set()
        self.mission_list = []
        self.ls_to_co_matrix = [[0 for _ in range(len(self.crossovers))] for _ in range(len(self.lock_stations))]
        self.exit_to_ls_matrix = [0 for _ in range(len(self.lock_stations))]
        self.ls_to_co_min = [0 for _ in range(len(self.crossovers))]
        self.qc_num, self.ls_num, self.is_num, self.yc_num, self.J_num_all = inst_type
        self.J_num = split_integer(self.J_num_all, self.qc_num)
        self.machine_num = self.qc_num + self.ls_num + self.is_num + self.yc_num
        self.machine_name_to_idx = self.match_machine_name_to_idx()

    def cal_finish_time(self):
        max_makespan = 0
        for yard_crane in self.yard_cranes.values():
            if yard_crane.process_time:
                if yard_crane.process_time[-1][-1] > max_makespan:
                    max_makespan = yard_crane.process_time[-1][-1]
        return max_makespan

    def cal_station_makespan(self):
        max_makespan = 0
        for lock_station in self.lock_stations.values():
            if any(lock_station.process_time):
                if lock_station.process_time[-1][-1] > max_makespan:
                    max_makespan = lock_station.process_time[-1][-1]
        return max_makespan

    def get_station_assign_list(self):
        assign_list = []
        for lock_station in self.lock_stations.values():
            station_assign_list = []
            for mission in lock_station.mission_list:
                station_assign_list.append(mission.idx[1:])
                print(mission.idx[1:], end='\t')
            assign_list.append(assign_list)
            print()

    def get_yard_cranes_set(self):
        yard_cranes_set = []
        for quay_crane in self.quay_cranes.values():
            for mission in quay_crane.missions.values():
                if mission.yard_block_loc[0] not in yard_cranes_set:
                    yard_cranes_set.append(mission.yard_block_loc[0])
        yard_cranes_set = sorted(yard_cranes_set)
        return yard_cranes_set

    def match_machine_name_to_idx(self):
        machine_name_to_idx = {}
        cnt = 0
        for i in range(self.qc_num):
            machine_name_to_idx['QC' + str(i + 1)] = cnt
            cnt = cnt + 1
        for i in range(self.ls_num):
            machine_name_to_idx['S' + str(i + 1)] = cnt
            cnt = cnt + 1
        for i in range(self.is_num):
            machine_name_to_idx['CO' + str(i + 1)] = cnt
            cnt = cnt + 1
        for yard_crane in self.yard_cranes_set:
            machine_name_to_idx['YC' + yard_crane] = cnt
            cnt = cnt + 1
        return machine_name_to_idx


def split_integer(m, n):
    assert n > 0
    quotient = int(m / n)
    remainder = m % n
    if remainder > 0:
        return [quotient] * (n - remainder) + [quotient + 1] * remainder
    if remainder < 0:
        return [quotient - 1] * -remainder + [quotient] * (n + remainder)
    return [quotient] * n


if __name__ == "main":
    pass
