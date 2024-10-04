#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：iter_solution.py
@Author  ：JacQ
@Date    ：2022/4/22 9:38
"""
from copy import deepcopy
from typing import Optional

import torch

import conf.configs as cf
from algorithm_factory.algo_utils import sort_missions
from algorithm_factory.algo_utils.machine_cal_methods import del_station_afterwards, del_machine, process_insert, \
    crossover_process_by_order, yard_crane_process_by_order, \
    process_init_solution_for_l2i, process_init_solution_for_l2a, quay_crane_release_mission
from algorithm_factory.algo_utils.mission_cal_methods import derive_mission_attribute_list
from algorithm_factory.algo_utils.operators import inter_relocate_latest_station_longest_mission_to_earliest_machine, \
    inter_swap_latest_station_random_mission_earliest_machine, \
    inter_swap_latest_station_longest_mission_earliest_machine, \
    inter_relocate_random_station_random_mission_to_random_machine, inner_relocate_latest_station_longest_mission, \
    inner_relocate_latest_yard_crane_random_mission
from algorithm_factory.algo_utils.rl_methods import del_tensor_ele
from common.port_env import PortEnv


class IterSolution:
    def __init__(self,
                 port_env: PortEnv):
        self.init_env: PortEnv = port_env
        self.iter_env: Optional[PortEnv] = None
        self.last_step_makespan: float = 0.0
        self.machine_chosen_num: list = [0] * len(self.init_env.lock_stations)
        self.attri_ls = None
        self.released_missions = []

    def reset(self):
        self.iter_env = deepcopy(self.init_env)
        self.released_missions = []

    def l2a_init(self):
        process_init_solution_for_l2a(self.init_env)
        self.iter_env = deepcopy(self.init_env)

    def l2i_init(self):
        process_init_solution_for_l2i(self.init_env)
        self.iter_env = deepcopy(self.init_env)
        self.last_step_makespan = self.iter_env.cal_finish_time()

    def ua_init(self):
        process_init_solution_for_l2a(self.init_env)
        self.iter_env = deepcopy(self.init_env)
        self.attri_ls = derive_mission_attribute_list(self.init_env.mission_list)

    def ua_n_init(self):
        process_init_solution_for_l2a(self.init_env)
        # quay_crane_process_by_order(self.init_env)
        getattr(sort_missions, "A_EXIT")(self.init_env.mission_list)
        self.iter_env = deepcopy(self.init_env)

    def step_v1(self, action):
        flag = 'not end'
        # print(action)
        if action == 0:
            flag = inter_relocate_random_station_random_mission_to_random_machine(self.iter_env)
        elif action == 1:
            flag = inter_relocate_latest_station_longest_mission_to_earliest_machine(self.iter_env)
        elif action == 2:
            flag = inter_swap_latest_station_random_mission_earliest_machine(self.iter_env)
        elif action == 3:
            flag = inter_swap_latest_station_longest_mission_earliest_machine(self.iter_env)
        elif action == 4:
            flag = inner_relocate_latest_station_longest_mission(self.iter_env)
        elif action == 5:
            flag = inner_relocate_latest_yard_crane_random_mission(self.iter_env)
        if flag == 'end':
            print('end action ' + str(action))

        new_makespan = self.iter_env.cal_finish_time()
        for mission in self.iter_env.mission_list:
            mission.cal_mission_attributes()
        self.last_step_makespan = new_makespan
        reward = 5000 / new_makespan * 1.0
        return reward, flag

    def step_v2(self, action, mission, step_number, buffer_flag=True):
        curr_station = list(self.iter_env.lock_stations.values())[action]
        quay_crane_release_mission(port_env=self.iter_env, mission=mission)
        del_station_afterwards(port_env=self.iter_env, buffer_flag=buffer_flag, step_number=step_number)
        del_machine(curr_station, buffer_flag)
        process_insert(mission, curr_station, buffer_flag)
        crossover_process_by_order(self.iter_env, buffer_flag, step_number + 1)
        yard_crane_process_by_order(self.iter_env, buffer_flag, step_number + 1)
        cur_makespan = self.iter_env.cal_finish_time()
        # station_makespan = self.cal_station_makespan()
        # makespan_delta = cur_makespan - self.last_step_makespan
        self.last_step_makespan = cur_makespan
        return cur_makespan

    def step_v22(self, action, mission, step_number, buffer_flag=True):
        curr_station = list(self.iter_env.lock_stations.values())[action]
        quay_crane_release_mission(port_env=self.iter_env, mission=mission)
        del_station_afterwards(port_env=self.iter_env, buffer_flag=buffer_flag, step_number=step_number,
                               released_mission_ls=self.released_missions)
        del_machine(curr_station, buffer_flag)
        process_insert(mission, curr_station, buffer_flag)
        crossover_process_by_order(self.iter_env, buffer_flag, step_number + 1,
                                   released_mission_ls=self.released_missions)
        yard_crane_process_by_order(self.iter_env, buffer_flag, step_number + 1,
                                    released_mission_ls=self.released_missions)
        cur_makespan = self.iter_env.cal_finish_time()
        # station_makespan = self.cal_station_makespan()
        # makespan_delta = cur_makespan - self.last_step_makespan
        self.last_step_makespan = cur_makespan
        return cur_makespan

    def step_v3(self, action: list, step: int, i: int, new_state: torch.Tensor,
                quay_num: int, quay_buffer_size: int, each_quay_m_num: int, m_max_num: int, m_attri_num: int,
                buffer_pro_time: float = cf.RELEASE_TIME):
        # move target to first pos & update release time
        first_pos = step
        cur_mission_pos = action[i] + step
        for j in range(first_pos, cur_mission_pos):
            temp_mission = self.iter_env.buffers['BF' + str(i + 1)].mission_list[j]
            temp_mission.release_time += buffer_pro_time
            temp_mission.machine_start_time = [temp_mission.machine_start_time[i] + buffer_pro_time for i in
                                               range(len(temp_mission.machine_start_time))]
        cur_mission = self.iter_env.buffers['BF' + str(i + 1)].mission_list[cur_mission_pos]
        cur_mission.release_time = cur_mission.release_time - action[i] * buffer_pro_time
        cur_mission.machine_start_time = [cur_mission.machine_start_time[k] - action[i] * buffer_pro_time for k
                                          in range(len(cur_mission.machine_start_time))]
        self.iter_env.buffers['BF' + str(i + 1)].mission_list.remove(cur_mission)
        self.iter_env.buffers['BF' + str(i + 1)].mission_list.insert(first_pos, cur_mission)
        # update vector u_state
        new_state = del_tensor_ele(new_state, action[i] + quay_num * i)
        if step >= each_quay_m_num - quay_buffer_size:
            new_slice = torch.zeros(m_attri_num).reshape(1, 1, -1)
        else:
            new_slice = torch.tensor(self.attri_ls['M' + str(
                step + i * each_quay_m_num + quay_buffer_size + 1)]).reshape(1, 1, -1)
        new_state = torch.cat(
            (new_state[:, 0:(i + 1) * quay_buffer_size - 1, :], new_slice,
             new_state[:, (i + 1) * quay_buffer_size - 1:, :]), 1)
        return cur_mission, new_state
