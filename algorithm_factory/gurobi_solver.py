# -*- coding: utf-8 -*-
# @Time    : 2022/9/8 10:01 PM
# @Author  : JacQ
# @File    : gurobi_solver.py
import time
from copy import deepcopy

from gurobipy import *  # 在Python中调用gurobi求解包
from algorithm_factory.algo_utils import sort_missions
from common.iter_solution import IterSolution
import conf.configs as cf


class CongestionPortModel:
    def __init__(self, solu: IterSolution):
        self.env = solu.init_env
        self.solu = solu
        self.S_num = 4
        self.J_num = self.env.J_num
        self.J_num_all = self.env.J_num_all
        self.K_num = -1
        self.big_M = 1000000
        self.S = [0, 1, 2, 3]
        self.K_s = []
        self.K = []
        self.J = []
        self.J_k = []
        self.A = []
        self.st = [[self.big_M for _ in range(self.J_num_all + 2)] for _ in range(self.J_num_all + 2)]
        self.pt = [[0 for _ in range(self.J_num_all + 2)] for _ in self.S]
        self.tt = [[[0 for _ in range(self.env.machine_num)] for _ in range(self.env.machine_num - self.env.yc_num)] for
                   _ in range(self.J_num_all + 2)]
        self.init_model()

        self.MLP = None
        self.v = None
        self.x = None
        self.C = None
        self.o = None
        self.r = None
        self.u = None
        self.alpha = None
        self.beta = None
        self.gamma = None
        self.gamma2 = None
        self.theta = None
        self.theta2 = None

    def init_model(self):
        self.K_num = self.env.machine_num
        self.big_M = 100000
        self.K_s = [[i for i in range(self.env.qc_num)],
                    [i + self.env.qc_num for i in range(self.env.ls_num)],
                    [i + self.env.qc_num + self.env.ls_num for i in range(self.env.is_num)],
                    [i + self.env.qc_num + self.env.ls_num + self.env.is_num for i in
                     range(self.env.yc_num)]]
        self.K = [i for i in range(self.env.machine_num)]
        self.J = [j for j in range(self.J_num_all)]
        self.J.append(self.J[-1] + 1)
        self.J.append(self.J[-1] + 1)

        tmp_mission_ls = deepcopy(self.env.mission_list)
        getattr(sort_missions, 'CHAR_ORDER')(tmp_mission_ls)

        self.J_k = [[] for _ in range(self.K_num)]
        for qc in self.env.quay_cranes.values():
            for mission in qc.missions.values():
                mission_idx = int(mission.idx[1:]) - 1
                qc_idx = self.env.machine_name_to_idx[mission.quay_crane_id]
                is_idx = self.env.machine_name_to_idx[mission.crossover_id]
                yc_idx = self.env.machine_name_to_idx['YC' + mission.yard_block_loc[0]]
                self.J_k[qc_idx].append(mission_idx)
                self.J_k[is_idx].append(mission_idx)
                self.J_k[yc_idx].append(mission_idx)
                if (mission_idx + 1) != sum(self.J_num[0:qc_idx + 1]):
                    self.A.append([mission_idx, mission_idx + 1])
                self.pt[0][mission_idx] = mission.machine_process_time[0]
                self.pt[1][mission_idx] = mission.station_process_time
                self.pt[2][mission_idx] = mission.intersection_process_time
                self.pt[3][mission_idx] = mission.yard_crane_process_time
        for k in self.J_k:
            if len(k) > 0:
                k.append(self.J[-2])
                k.append(self.J[-1])
        for i in range(len(self.J) - 2):
            mission_i = tmp_mission_ls[i]
            for j in range(i, len(self.J)):
                if j == len(self.J) - 2:
                    self.st[j][i] = abs(mission_i.yard_block_loc[1]) * cf.SLOT_LENGTH / cf.YARDCRANE_SPEED_X + \
                                    abs(cf.SLOT_NUM_Y - mission_i.yard_block_loc[2]) * cf.SLOT_WIDTH \
                                    / cf.YARDCRANE_SPEED_Y * 2
                    self.st[j][-1] = 0
                elif j == len(self.J) - 1:
                    self.st[i][j] = 0
                else:
                    mission_j = tmp_mission_ls[j]
                    if mission_i.yard_block_loc[0] == mission_j.yard_block_loc[0] and mission_i != mission_j:
                        self.st[i][j] = abs(mission_i.yard_block_loc[1] - mission_j.yard_block_loc[1]) * cf.SLOT_LENGTH \
                                        / cf.YARDCRANE_SPEED_X + \
                                        abs(mission_j.yard_block_loc[2] - cf.SLOT_NUM_Y) * cf.SLOT_WIDTH \
                                        / cf.YARDCRANE_SPEED_Y * 2
                        self.st[j][i] = abs(mission_i.yard_block_loc[1] - mission_j.yard_block_loc[1]) * cf.SLOT_LENGTH \
                                        / cf.YARDCRANE_SPEED_X + \
                                        abs(mission_i.yard_block_loc[2] - cf.SLOT_NUM_Y) * cf.SLOT_WIDTH \
                                        / cf.YARDCRANE_SPEED_Y * 2
                    else:
                        self.st[i][j] = self.big_M
                        self.st[j][i] = self.st[i][j]
        for j in range(len(self.J)):
            if j != len(self.J) - 1 and j != len(self.J) - 2:
                mission = tmp_mission_ls[j]
                for ls in range(self.env.ls_num):
                    self.tt[j][self.env.machine_name_to_idx[mission.quay_crane_id]][
                        self.env.machine_name_to_idx['S' + str(ls + 1)]] = \
                        self.env.quay_cranes[mission.quay_crane_id].time_to_exit + \
                        self.env.exit_to_ls_matrix[ls] / mission.vehicle_speed
                    self.tt[j][self.env.machine_name_to_idx['S' + str(ls + 1)]][
                        self.env.machine_name_to_idx[mission.crossover_id]] = \
                        self.env.ls_to_co_matrix[ls][int(mission.crossover_id[-1]) - 1] / mission.vehicle_speed
                self.tt[j][self.env.machine_name_to_idx[mission.crossover_id]][
                    self.env.machine_name_to_idx['YC' + mission.yard_block_loc[0]]] = tmp_mission_ls[
                    j].transfer_time_c2y
            else:
                for qc in range(self.env.qc_num):
                    for ls in range(self.env.ls_num):
                        self.tt[j][qc][ls + self.env.qc_num] = 0
                for ls in range(self.env.ls_num):
                    for co in range(self.env.is_num):
                        self.tt[j][ls + self.env.qc_num][co + self.env.qc_num + self.env.ls_num] = 0
                for co in range(self.env.is_num):
                    for yc in range(self.env.yc_num):
                        self.tt[j][co + self.env.qc_num + self.env.ls_num][
                            yc + self.env.qc_num + self.env.ls_num + self.env.is_num] = 0

    def construct(self):
        self.MLP = Model("port operation")
        # v_jk
        self.v = [[[] for _ in self.K] for _ in self.J]
        for j in self.J:
            for k in self.K:
                name = 'v_' + str(j) + "_" + str(k)
                self.v[j][k] = self.MLP.addVar(0, 1, vtype=GRB.BINARY, name=name)
        # x_ijk
        self.x = [[[[] for _ in self.K] for _ in self.J] for _ in self.J]
        for j in self.J:
            for jj in self.J:
                for k in self.K:
                    name = 'x_' + str(j) + "_" + str(jj) + "_" + str(k)
                    self.x[j][jj][k] = self.MLP.addVar(0, 1, vtype=GRB.BINARY, name=name)
        # C^s_j
        self.C = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'C_' + str(s) + "_" + str(j)
                self.C[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # o^s_j
        self.o = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'o_' + str(s) + "_" + str(j)
                self.o[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # r_j
        self.r = []
        for j in self.J:
            name = 'r_' + str(j)
            self.r.append(self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name))
        # u^s_j
        self.u = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'u_' + str(s) + "_" + str(j)
                self.u[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # alpha
        self.alpha = [[0 for _ in self.J] for _ in range(4)]
        for i in range(4):
            for j in self.J:
                name = 'alpha_' + str(i) + "_" + str(j)
                self.alpha[i][j] = self.MLP.addVar(0, 1, vtype=GRB.BINARY, name=name)
        # beta
        self.beta = [[0 for _ in self.J] for _ in range(4)]
        for i in range(4):
            for j in self.J:
                name = 'beta_' + str(i) + "_" + str(j)
                self.beta[i][j] = self.MLP.addVar(0, 1, vtype=GRB.BINARY, name=name)

        self.MLP.addConstrs((self.x[j][j][k] == 0 for k in self.K for j in self.J), "con0")
        self.MLP.addConstrs((sum(self.v[j][k] for k in self.K_s[s]) == 1 for j in self.J[0:-2] for s in self.S), "con3")
        self.MLP.addConstrs((self.C[s + 1][j] >=
                             self.o[s + 1][j] + self.C[s][j] + self.u[s][j] for s in range(3) for j in self.J), "con4")
        for s in range(1, 4):
            for k in self.K_s[s]:
                for j in self.J:
                    for jj in self.J:
                        self.MLP.addConstr(
                            self.C[s][jj] - self.o[s][jj] - self.C[s][j] + self.big_M - self.big_M *
                            self.x[j][jj][k] >= 0, "con5")
        self.MLP.addConstrs((self.C[0][j] == self.r[j] + self.o[0][j] for j in self.J), "con9")
        tmp_J = self.J[0:-2].copy()
        tmp_J.append(self.J[-1])
        self.MLP.addConstrs(
            (sum(self.x[jj][j][k] for jj in self.J) == self.v[j][k] for j in tmp_J for k in self.K), "con10")
        self.MLP.addConstrs(
            (sum(self.x[j][jj][k] for jj in self.J) == self.v[j][k] for j in self.J[0:-1] for k in self.K), "con11")
        self.MLP.addConstrs(
            (self.C[s][jj] + self.u[s][jj] + self.big_M - self.big_M * self.x[j][jj][k] >= self.C[s][j] + self.u[s][
                j] for j in self.J for jj in self.J for s in range(3) for k in self.K_s[s + 1]), "con12")
        self.MLP.addConstrs(
            (self.C[s + 1][j] - self.o[s + 1][j] <= self.C[s][j] + self.u[s][j] + self.big_M - self.big_M *
             self.alpha[s + 1][j] for s in range(0, 3) for j in self.J[0:-2]), "con14")
        self.MLP.addConstrs(
            (self.C[s][j] - self.o[s][j] <= self.C[s][jj] + 2 * self.big_M - self.big_M * self.beta[s][
                j] - self.big_M * self.x[jj][j][k] for s in range(1, 4) for k in self.K_s[s] for j in self.J[0:-2]
             for jj in self.J), "con15")
        self.MLP.addConstrs((self.alpha[s][j] + self.beta[s][j] >= 1 for s in range(1, 4) for j in self.J[0:-2]),
                            "con16")
        self.MLP.addConstrs((self.o[s][-1] == 0 for s in self.S), "con202")
        self.MLP.addConstrs((self.o[s][-2] == 0 for s in self.S), "con203")
        self.MLP.addConstrs((self.u[s][self.J[-1]] == 0 for s in range(0, 3)), "con190")
        self.MLP.addConstrs((self.u[s][self.J[-2]] == 0 for s in range(0, 3)), "con191")

        tmp_mission_ls = deepcopy(self.env.mission_list)
        getattr(sort_missions, 'CHAR_ORDER')(tmp_mission_ls)
        self.MLP.addConstrs((self.v[j][k] == 1 for k in self.K for j in self.J_k[k]), "con8")
        for pair in self.A:
            j, jj = pair[0], pair[1]
            self.MLP.addConstr(self.r[jj] - self.C[0][j] >= 0, "con10" + str(j) + str(jj))
            k = int(tmp_mission_ls[j].quay_crane_id[-1]) - 1
            self.MLP.addConstr(self.x[j][jj][k] == 1, "con11" + str(j) + str(jj))
        self.MLP.addConstr(self.r[-2] == 0, "con00")
        self.MLP.addConstrs((self.o[s][j] == self.pt[s][j] for s in range(0, 3) for j in self.J[0:-2]), "con200")
        self.MLP.addConstrs((self.o[3][j] == self.pt[3][j] + sum(
            self.x[jj][j][k] * self.st[jj][j] for jj in self.J for k in self.K_s[3]) for j in self.J[0:-2]), "con201")
        self.MLP.addConstrs(
            (self.u[s][j] - self.tt[j][k][kk] + self.big_M * 2 - self.big_M * self.v[j][k] - self.big_M *
             self.v[j][kk] >= 0 for j in self.J[0:-2] for s in range(0, 3) for k in self.K_s[s] for kk in
             self.K_s[s + 1]), "con17")
        self.MLP.addConstrs(
            (self.u[s][j] - self.tt[j][k][kk] <= self.big_M * 2 - self.big_M * self.v[j][k] - self.big_M *
             self.v[j][kk] for j in self.J[0:-2] for s in range(0, 3) for k in self.K_s[s] for kk in self.K_s[s + 1]),
            "con18")

        q_1 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_1')
        q_2 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_2')
        q_3 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_3')
        q_4 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_4')
        q_5 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_5')
        self.MLP.addConstrs((q_1 >= self.C[3][j] for j in self.J), "obj1")
        self.MLP.addConstr((q_2 == sum(self.C[s + 1][j] - self.o[s + 1][j]
                                       - self.C[s][j] - self.u[s][j] for j in tmp_J for s in range(0, 3))), "obj2")
        self.MLP.addConstr((q_3 == sum(self.C[s + 1][j] - self.o[s + 1][j]
                                       - self.C[s][j] - self.u[s][j] for j in tmp_J for s in [1, 2])), "obj3")
        self.MLP.addConstr((q_4 == sum(self.C[s + 1][j] - self.o[s + 1][j]
                                       - self.C[s][j] - self.u[s][j] for j in tmp_J for s in [0, 2])), "obj4")
        self.MLP.addConstr((q_5 == sum(self.C[s + 1][j] - self.o[s + 1][j]
                                       - self.C[s][j] - self.u[s][j] for j in tmp_J for s in [0, 1])), "obj5")
        self.MLP.addConstr(q_1 <= self.gamma, "obj1")
        # self.MLP.addConstr(q_1 >= self.gamma2, "obj1")
        self.MLP.setObjective(q_2, GRB.MINIMIZE)
        self.MLP.update()


class CongestionPortModel2:
    def __init__(self, solu: IterSolution):
        self.env = solu.init_env
        self.solu = solu
        self.S_num = 4
        self.J_num = self.env.J_num
        self.J_num_all = self.env.J_num_all
        self.K_num = -1
        self.big_M = 1000000
        self.S = [0, 1, 2, 3]
        self.K_s = []
        self.K = []
        self.J = []
        self.J_k = []
        self.A = []
        self.st = [[self.big_M for _ in range(self.J_num_all + 2)] for _ in range(self.J_num_all + 2)]
        self.pt = [[0 for _ in range(self.J_num_all + 2)] for _ in self.S]
        self.tt = [[[0 for _ in range(self.env.machine_num)] for _ in range(self.env.machine_num - self.env.yc_num)] for
                   _ in range(self.J_num_all + 2)]
        self.init_model()

        self.MLP = None
        self.v = None  # v_jk
        self.x = None  # x_ijk
        self.C = None  # C^s_j
        self.o = None  # o^s_j
        self.r = None  # r_j
        self.u = None  # u^s_j
        self.alpha = None
        self.beta = None
        self.gamma = None
        self.gamma2 = None
        self.theta = None
        self.theta2 = None

    def init_model(self):
        self.K_num = self.env.machine_num
        self.big_M = 100000
        self.K_s = [[i for i in range(self.env.qc_num)],
                    [i + self.env.qc_num for i in range(self.env.ls_num)],
                    [i + self.env.qc_num + self.env.ls_num for i in range(self.env.is_num)],
                    [i + self.env.qc_num + self.env.ls_num + self.env.is_num for i in
                     range(self.env.yc_num)]]
        self.K = [i for i in range(self.env.machine_num)]
        self.J = [j for j in range(self.J_num_all)]
        self.J.append(self.J[-1] + 1)  # dummy job N+1
        self.J.append(self.J[-1] + 1)  # dummy job N+2
        tmp_mission_ls = deepcopy(self.env.mission_list)
        getattr(sort_missions, 'CHAR_ORDER')(tmp_mission_ls)
        # J_K
        self.J_k = [[] for _ in range(self.K_num)]
        for qc in self.env.quay_cranes.values():
            for mission in qc.missions.values():
                mission_idx = int(mission.idx[1:]) - 1
                qc_idx = self.env.machine_name_to_idx[mission.quay_crane_id]
                is_idx = self.env.machine_name_to_idx[mission.crossover_id]
                yc_idx = self.env.machine_name_to_idx['YC' + mission.yard_block_loc[0]]
                self.J_k[qc_idx].append(mission_idx)
                self.J_k[is_idx].append(mission_idx)
                self.J_k[yc_idx].append(mission_idx)
                if (mission_idx + 1) != sum(self.J_num[0:qc_idx + 1]):
                    self.A.append([mission_idx, mission_idx + 1])
                self.pt[0][mission_idx] = mission.machine_process_time[0]
                self.pt[1][mission_idx] = mission.station_process_time
                self.pt[2][mission_idx] = mission.intersection_process_time
                self.pt[3][mission_idx] = mission.yard_crane_process_time
        for k in self.J_k:
            if len(k) > 0:
                k.append(self.J[-2])
                k.append(self.J[-1])
        for i in range(len(self.J) - 2):
            mission_i = tmp_mission_ls[i]
            for j in range(i, len(self.J)):
                if j == len(self.J) - 2:
                    self.st[j][i] = abs(mission_i.yard_block_loc[1]) * cf.SLOT_LENGTH / cf.YARDCRANE_SPEED_X + \
                                    abs(cf.SLOT_NUM_Y - mission_i.yard_block_loc[2]) * cf.SLOT_WIDTH \
                                    / cf.YARDCRANE_SPEED_Y * 2
                    self.st[j][-1] = 0
                elif j == len(self.J) - 1:
                    self.st[i][j] = 0
                else:
                    mission_j = tmp_mission_ls[j]
                    if mission_i.yard_block_loc[0] == mission_j.yard_block_loc[0] and mission_i != mission_j:
                        self.st[i][j] = abs(mission_i.yard_block_loc[1] - mission_j.yard_block_loc[1]) * cf.SLOT_LENGTH \
                                        / cf.YARDCRANE_SPEED_X + \
                                        abs(mission_j.yard_block_loc[2] - cf.SLOT_NUM_Y) * cf.SLOT_WIDTH \
                                        / cf.YARDCRANE_SPEED_Y * 2
                        self.st[j][i] = abs(mission_i.yard_block_loc[1] - mission_j.yard_block_loc[1]) * cf.SLOT_LENGTH \
                                        / cf.YARDCRANE_SPEED_X + \
                                        abs(mission_i.yard_block_loc[2] - cf.SLOT_NUM_Y) * cf.SLOT_WIDTH \
                                        / cf.YARDCRANE_SPEED_Y * 2
                    else:
                        self.st[i][j] = self.big_M
                        self.st[j][i] = self.st[i][j]
        for j in range(len(self.J)):
            if j != len(self.J) - 1 and j != len(self.J) - 2:
                mission = tmp_mission_ls[j]
                for ls in range(self.env.ls_num):
                    self.tt[j][self.env.machine_name_to_idx[mission.quay_crane_id]][
                        self.env.machine_name_to_idx['S' + str(ls + 1)]] = \
                        self.env.quay_cranes[mission.quay_crane_id].time_to_exit + \
                        self.env.exit_to_ls_matrix[ls] / mission.vehicle_speed
                    self.tt[j][self.env.machine_name_to_idx['S' + str(ls + 1)]][
                        self.env.machine_name_to_idx[mission.crossover_id]] = \
                        self.env.ls_to_co_matrix[ls][int(mission.crossover_id[-1]) - 1] / mission.vehicle_speed
                self.tt[j][self.env.machine_name_to_idx[mission.crossover_id]][
                    self.env.machine_name_to_idx['YC' + mission.yard_block_loc[0]]] = tmp_mission_ls[
                    j].transfer_time_c2y
            else:
                for qc in range(self.env.qc_num):
                    for ls in range(self.env.ls_num):
                        self.tt[j][qc][ls + self.env.qc_num] = 0
                for ls in range(self.env.ls_num):
                    for co in range(self.env.is_num):
                        self.tt[j][ls + self.env.qc_num][co + self.env.qc_num + self.env.ls_num] = 0
                for co in range(self.env.is_num):
                    for yc in range(self.env.yc_num):
                        self.tt[j][co + self.env.qc_num + self.env.ls_num][
                            yc + self.env.qc_num + self.env.ls_num + self.env.is_num] = 0

    def construct(self, solved_env: IterSolution):
        self.MLP = Model("port operation")
        # C^s_j
        self.C = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'C_' + str(s) + "_" + str(j)
                self.C[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # o^s_j
        self.o = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'o_' + str(s) + "_" + str(j)
                self.o[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # r_j
        self.r = []
        for j in self.J:
            name = 'r_' + str(j)
            self.r.append(self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name))
        # u^s_j
        self.u = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'u_' + str(s) + "_" + str(j)
                self.u[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # alpha
        self.alpha = [[0 for _ in self.J] for _ in range(4)]
        for i in range(4):
            for j in self.J:
                name = 'alpha_' + str(i) + "_" + str(j)
                self.alpha[i][j] = self.MLP.addVar(0, 1, vtype=GRB.BINARY, name=name)
        # beta
        self.beta = [[0 for _ in self.J] for _ in range(4)]
        for i in range(4):
            for j in self.J:
                name = 'beta_' + str(i) + "_" + str(j)
                self.beta[i][j] = self.MLP.addVar(0, 1, vtype=GRB.BINARY, name=name)
        # v_jk
        self.v = [[0 for _ in self.K] for _ in self.J]
        # x_ijk
        self.x = [[[0 for _ in self.K] for _ in self.J] for _ in self.J]
        J_num_all = solved_env.iter_env.J_num_all
        ls = solved_env.iter_env.mission_list
        for j in range(len(solved_env.iter_env.mission_list)):
            m_idx = int(ls[j].machine_list[4][-1]) + solved_env.iter_env.qc_num - 1
            qc_idx = int(ls[j].quay_crane_id[2:]) - 1
            self.v[int(ls[j].idx[1:]) - 1][m_idx] = 1
            self.v[int(ls[j].idx[1:]) - 1][qc_idx] = 1
        for i in range(len(solved_env.iter_env.quay_cranes)):
            self.v[J_num_all][i] = 1
            self.v[J_num_all + 1][i] = 1
        tmp_mission_ls = deepcopy(self.env.mission_list)
        for pair in self.A:
            j, jj = pair[0], pair[1]
            self.MLP.addConstr(self.r[jj] - self.C[0][j] >= 0, "con10" + str(j) + str(jj))
            k = int(tmp_mission_ls[j].quay_crane_id[-1]) - 1
            self.x[j][jj][k] = 1
        for i in range(solved_env.iter_env.ls_num):
            ls_idx = solved_env.init_env.machine_name_to_idx['S' + str(i + 1)]
            getattr(sort_missions, "A_STATION_NB")(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list)
            self.v[J_num_all][ls_idx] = 1
            self.v[J_num_all + 1][ls_idx] = 1
            for j in range(len(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list)):
                p_mission_idx = int(
                    solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list[j].idx[1:]) - 1
                if j == 0 and j == len(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list) - 1:
                    self.x[J_num_all][p_mission_idx][ls_idx] = 1
                    self.x[p_mission_idx][0][J_num_all + 1] = 1
                elif j == 0:
                    l_mission_idx = \
                        int(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                    self.x[J_num_all][p_mission_idx][ls_idx] = 1
                    self.x[p_mission_idx][l_mission_idx][ls_idx] = 1
                elif j == len(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list) - 1:
                    self.x[p_mission_idx][J_num_all + 1][ls_idx] = 1
                else:
                    l_mission_idx = int(
                        solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                    self.x[p_mission_idx][l_mission_idx][ls_idx] = 1
        for i in range(len(solved_env.iter_env.crossovers)):
            co_idx = solved_env.init_env.machine_name_to_idx['CO' + str(i + 1)]
            self.v[J_num_all][co_idx] = 1
            self.v[J_num_all + 1][co_idx] = 1
            getattr(sort_missions, "A_CROSSOVER_NB")(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list)
            for j in range(len(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list)):
                p_mission_idx = int(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list[j].idx[1:]) - 1
                self.v[p_mission_idx][co_idx] = 1
                if j == 0 and j == len(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list) - 1:
                    self.x[J_num_all][p_mission_idx][co_idx] = 1
                    self.x[p_mission_idx][J_num_all + 1][co_idx] = 1
                elif j == 0:
                    l_mission_idx = int(
                        solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                    self.x[J_num_all][p_mission_idx][co_idx] = 1
                    self.x[p_mission_idx][l_mission_idx][co_idx] = 1
                elif j == len(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list) - 1:
                    self.x[p_mission_idx][J_num_all + 1][co_idx] = 1
                else:
                    l_mission_idx = int(
                        solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                    self.x[p_mission_idx][l_mission_idx][co_idx] = 1
        for yc in solved_env.init_env.yard_cranes_set:
            yc_idx = solved_env.init_env.machine_name_to_idx['YC' + yc]
            self.v[J_num_all][yc_idx] = 1
            self.v[J_num_all + 1][yc_idx] = 1
            getattr(sort_missions, "A_YARD_NB")(solved_env.iter_env.yard_cranes['YC' + yc].mission_list)
            for j in range(len(solved_env.iter_env.yard_cranes['YC' + yc].mission_list)):
                p_mission_idx = int(solved_env.iter_env.yard_cranes['YC' + yc].mission_list[j].idx[1:]) - 1
                self.v[p_mission_idx][yc_idx] = 1
                if j == 0 and j == len(solved_env.iter_env.yard_cranes['YC' + yc].mission_list) - 1:
                    self.x[J_num_all][p_mission_idx][yc_idx] = 1
                    self.x[p_mission_idx][J_num_all + 1][yc_idx] = 1
                elif j == 0:
                    l_mission_idx = int(solved_env.iter_env.yard_cranes['YC' + yc].mission_list[j + 1].idx[1:]) - 1
                    self.x[J_num_all][p_mission_idx][yc_idx] = 1
                    self.x[p_mission_idx][l_mission_idx][yc_idx] = 1
                elif j == len(solved_env.iter_env.yard_cranes['YC' + yc].mission_list) - 1:
                    self.x[p_mission_idx][J_num_all + 1][yc_idx] = 1
                else:
                    l_mission_idx = int(solved_env.iter_env.yard_cranes['YC' + yc].mission_list[j + 1].idx[1:]) - 1
                    self.x[p_mission_idx][l_mission_idx][yc_idx] = 1
        print("variable worked")
        tmp_J = self.J[0:-2].copy()
        tmp_J.append(self.J[-1])

        self.MLP.addConstrs((self.C[s + 1][j] >=
                             self.o[s + 1][j] + self.C[s][j] + self.u[s][j] for s in range(3) for j in self.J), "con4")
        self.MLP.addConstrs((self.C[s][jj] - self.o[s][jj] - self.C[s][j] >= 0 for s in range(1, 4)
                             for k in self.K_s[s] for j in self.J for jj in self.J if self.x[j][jj][k] == 1), "con5")
        self.MLP.addConstrs((self.C[0][j] == self.r[j] + self.o[0][j] for j in self.J), "con9")

        self.MLP.addConstrs(
            (self.C[s][jj] + self.u[s][jj] >= self.C[s][j] + self.u[s][j] for j in self.J for jj in self.J for s in
             range(3) for k in self.K_s[s + 1] if self.x[j][jj][k] == 1), "con12")
        self.MLP.addConstrs(
            (self.C[s + 1][j] - self.o[s + 1][j] <= self.C[s][j] + self.u[s][j] + self.big_M - self.big_M *
             self.alpha[s + 1][j] for s in range(0, 3) for j in self.J[0:-2]), "con14")
        self.MLP.addConstrs(
            (self.C[s][j] - self.o[s][j] <= self.C[s][jj] + 2 * self.big_M - self.big_M * self.beta[s][j] - self.big_M
             for s in range(1, 4) for k in self.K_s[s] for j in self.J[0:-2] for jj in self.J if self.x[jj][j][k] == 1),
            "con15")
        self.MLP.addConstrs((self.alpha[s][j] + self.beta[s][j] >= 1 for s in range(1, 4) for j in self.J[0:-2]),
                            "con16")
        self.MLP.addConstrs((self.o[s][-1] == 0 for s in self.S), "con202")
        self.MLP.addConstrs((self.o[s][-2] == 0 for s in self.S), "con203")
        self.MLP.addConstrs((self.u[s][self.J[-1]] == 0 for s in range(0, 3)), "con190")
        self.MLP.addConstrs((self.u[s][self.J[-2]] == 0 for s in range(0, 3)), "con191")
        self.MLP.addConstr(self.r[-2] == 0, "con00")
        self.MLP.addConstrs((self.o[s][j] == self.pt[s][j] for s in range(0, 3) for j in self.J[0:-2]), "con200")
        self.MLP.addConstrs((self.o[3][j] == self.pt[3][j] + sum(
            self.x[jj][j][k] * self.st[jj][j] for jj in self.J for k in self.K_s[3]) for j in self.J[0:-2]), "con201")
        self.MLP.addConstrs((self.u[s][j] == self.tt[j][k][kk] for j in self.J[0:-2] for s in range(0, 3) for k in
                             self.K_s[s] for kk in self.K_s[s + 1] if self.v[j][k] == 1 and self.v[j][kk] == 1),
                            "con17")
        q_1 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_1')
        q_2 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_2')
        self.MLP.addConstrs((q_1 >= self.C[3][j] for j in self.J), "obj1")
        self.MLP.addConstr((q_2 == sum(self.C[s + 1][j] - self.o[s + 1][j]
                                       - self.C[s][j] - self.u[s][j] for j in tmp_J for s in range(0, 3))), "obj2")
        self.MLP.addConstr(q_1 <= self.gamma, "obj1")
        self.MLP.setObjective(q_2, GRB.MINIMIZE)
        self.MLP.update()


class RelaxedCongestionPortModel:
    def __init__(self, solu: IterSolution):
        self.env = solu.init_env
        self.solu = solu
        self.S_num = 4
        self.J_num = self.env.J_num
        self.J_num_all = self.env.J_num_all
        self.K_num = -1
        self.big_M = 1000000
        self.S = [0, 1, 2, 3]
        self.K_s = []
        self.K = []
        self.J = []
        self.J_k = []
        self.A = []
        self.st = [[self.big_M for _ in range(self.J_num_all + 2)] for _ in range(self.J_num_all + 2)]
        self.pt = [[0 for _ in range(self.J_num_all + 2)] for _ in self.S]
        self.tt = [[[0 for _ in range(self.env.machine_num)] for _ in range(self.env.machine_num - self.env.qc_num)] for
                   _ in range(self.J_num_all + 2)]
        self.init_model()

        self.MLP = None
        self.v = None  # v_jk
        self.x = None  # x_ijk
        self.C = None  # C^s_j
        self.o = None  # o^s_j
        self.r = None  # r_j
        self.u = None  # u^s_j
        self.alpha = None
        self.beta = None
        self.gamma = None
        self.gamma2 = None
        self.theta = None
        self.theta2 = None

    def init_model(self):
        self.K_num = self.env.machine_num
        self.big_M = 100000
        self.K_s = [[i for i in range(self.env.qc_num)],
                    [i + self.env.qc_num for i in range(self.env.ls_num)],
                    [i + self.env.qc_num + self.env.ls_num for i in range(self.env.is_num)],
                    [i + self.env.qc_num + self.env.ls_num + self.env.is_num for i in
                     range(self.env.yc_num)]]
        self.K = [i for i in range(self.env.machine_num)]
        self.J = [j for j in range(self.J_num_all)]
        self.J.append(self.J[-1] + 1)  # dummy job N+1
        self.J.append(self.J[-1] + 1)  # dummy job N+2
        tmp_mission_ls = deepcopy(self.env.mission_list)
        getattr(sort_missions, 'CHAR_ORDER')(tmp_mission_ls)
        self.J_k = [[] for _ in range(self.K_num)]
        for qc in self.env.quay_cranes.values():
            for mission in qc.missions.values():
                mission_idx = int(mission.idx[1:]) - 1
                qc_idx = self.env.machine_name_to_idx[mission.quay_crane_id]
                is_idx = self.env.machine_name_to_idx[mission.crossover_id]
                yc_idx = self.env.machine_name_to_idx['YC' + mission.yard_block_loc[0]]
                self.J_k[qc_idx].append(mission_idx)
                self.J_k[is_idx].append(mission_idx)
                self.J_k[yc_idx].append(mission_idx)
                if (mission_idx + 1) != sum(self.J_num[0:qc_idx + 1]):
                    self.A.append([mission_idx, mission_idx + 1])
                self.pt[0][mission_idx] = mission.machine_process_time[0]
                self.pt[1][mission_idx] = mission.station_process_time
                self.pt[2][mission_idx] = mission.intersection_process_time
                self.pt[3][mission_idx] = mission.yard_crane_process_time
        for k in self.J_k:
            if len(k) > 0:
                k.append(self.J[-2])
                k.append(self.J[-1])
        for i in range(len(self.J) - 2):
            mission_i = tmp_mission_ls[i]
            for j in range(i, len(self.J)):
                if j == len(self.J) - 2:
                    self.st[j][i] = abs(mission_i.yard_block_loc[1]) * cf.SLOT_LENGTH / cf.YARDCRANE_SPEED_X + \
                                    abs(cf.SLOT_NUM_Y - mission_i.yard_block_loc[2]) * cf.SLOT_WIDTH \
                                    / cf.YARDCRANE_SPEED_Y * 2
                    self.st[j][-1] = 0
                elif j == len(self.J) - 1:
                    self.st[i][j] = 0
                else:
                    mission_j = tmp_mission_ls[j]
                    if mission_i.yard_block_loc[0] == mission_j.yard_block_loc[0] and mission_i != mission_j:
                        self.st[i][j] = abs(mission_i.yard_block_loc[1] - mission_j.yard_block_loc[1]) * cf.SLOT_LENGTH \
                                        / cf.YARDCRANE_SPEED_X + \
                                        abs(mission_j.yard_block_loc[2] - cf.SLOT_NUM_Y) * cf.SLOT_WIDTH \
                                        / cf.YARDCRANE_SPEED_Y * 2
                        self.st[j][i] = abs(mission_i.yard_block_loc[1] - mission_j.yard_block_loc[1]) * cf.SLOT_LENGTH \
                                        / cf.YARDCRANE_SPEED_X + \
                                        abs(mission_i.yard_block_loc[2] - cf.SLOT_NUM_Y) * cf.SLOT_WIDTH \
                                        / cf.YARDCRANE_SPEED_Y * 2
                    else:
                        self.st[i][j] = self.big_M
                        self.st[j][i] = self.st[i][j]
        for j in range(len(self.J)):
            if j != len(self.J) - 1 and j != len(self.J) - 2:
                mission = tmp_mission_ls[j]
                for ls in range(self.env.ls_num):
                    self.tt[j][self.env.machine_name_to_idx[mission.quay_crane_id]][
                        self.env.machine_name_to_idx['S' + str(ls + 1)]] = \
                        self.env.quay_cranes[mission.quay_crane_id].time_to_exit + \
                        self.env.exit_to_ls_matrix[ls] / mission.vehicle_speed
                    self.tt[j][self.env.machine_name_to_idx['S' + str(ls + 1)]][
                        self.env.machine_name_to_idx[mission.crossover_id]] = \
                        self.env.ls_to_co_matrix[ls][int(mission.crossover_id[-1]) - 1] / mission.vehicle_speed
                self.tt[j][self.env.machine_name_to_idx[mission.crossover_id]][
                    self.env.machine_name_to_idx['YC' + mission.yard_block_loc[0]]] = tmp_mission_ls[
                    j].transfer_time_c2y
            else:
                for qc in range(self.env.qc_num):
                    for ls in range(self.env.ls_num):
                        self.tt[j][qc][ls + self.env.qc_num] = 0
                for ls in range(self.env.ls_num):
                    for co in range(self.env.is_num):
                        self.tt[j][ls + self.env.qc_num][co + self.env.qc_num + self.env.ls_num] = 0
                for co in range(self.env.is_num):
                    for yc in range(self.env.yc_num):
                        self.tt[j][co + self.env.qc_num + self.env.ls_num][
                            yc + self.env.qc_num + self.env.ls_num + self.env.is_num] = 0

    def construct(self):
        self.MLP = Model("port operation")
        # v_jk
        self.v = [[[] for _ in self.K] for _ in self.J]
        for j in self.J:
            for k in self.K:
                name = 'v_' + str(j) + "_" + str(k)
                self.v[j][k] = self.MLP.addVar(0, 1, vtype=GRB.CONTINUOUS, name=name)
        # x_ijk
        self.x = [[[[] for _ in self.K] for _ in self.J] for _ in self.J]
        for j in self.J:
            for jj in self.J:
                for k in self.K:
                    name = 'x_' + str(j) + "_" + str(jj) + "_" + str(k)
                    self.x[j][jj][k] = self.MLP.addVar(0, 1, vtype=GRB.CONTINUOUS, name=name)
        # C^s_j
        self.C = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'C_' + str(s) + "_" + str(j)
                self.C[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # o^s_j
        self.o = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'o_' + str(s) + "_" + str(j)
                self.o[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # r_j
        self.r = []
        for j in self.J:
            name = 'r_' + str(j)
            self.r.append(self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name))
        # u^s_j
        self.u = [[[] for _ in self.J] for _ in self.S]
        for s in self.S:
            for j in self.J:
                name = 'u_' + str(s) + "_" + str(j)
                self.u[s][j] = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name=name)
        # alpha
        self.alpha = [[0 for _ in self.J] for _ in range(4)]
        for i in range(4):
            for j in self.J:
                name = 'alpha_' + str(i) + "_" + str(j)
                self.alpha[i][j] = self.MLP.addVar(0, 1, vtype=GRB.CONTINUOUS, name=name)
        # beta
        self.beta = [[0 for _ in self.J] for _ in range(4)]
        for i in range(4):
            for j in self.J:
                name = 'beta_' + str(i) + "_" + str(j)
                self.beta[i][j] = self.MLP.addVar(0, 1, vtype=GRB.CONTINUOUS, name=name)

        self.MLP.addConstrs((self.x[j][j][k] == 0 for k in self.K for j in self.J), "con0")
        self.MLP.addConstrs((sum(self.v[j][k] for k in self.K_s[s]) == 1 for j in self.J[0:-2] for s in self.S), "con3")
        self.MLP.addConstrs((self.C[s + 1][j] >=
                             self.o[s + 1][j] + self.C[s][j] + self.u[s][j] for s in range(3) for j in self.J), "con4")
        for s in range(1, 4):
            for k in self.K_s[s]:
                for j in self.J:
                    for jj in self.J:
                        self.MLP.addConstr(
                            self.C[s][jj] - self.o[s][jj] - self.C[s][j] + self.big_M - self.big_M *
                            self.x[j][jj][k] >= 0, "con5")
        self.MLP.addConstrs((self.C[0][j] == self.r[j] + self.o[0][j] for j in self.J), "con9")
        tmp_J = self.J[0:-2].copy()
        tmp_J.append(self.J[-1])
        self.MLP.addConstrs(
            (sum(self.x[jj][j][k] for jj in self.J) == self.v[j][k] for j in tmp_J for k in self.K), "con10")
        self.MLP.addConstrs(
            (sum(self.x[j][jj][k] for jj in self.J) == self.v[j][k] for j in self.J[0:-1] for k in self.K), "con11")
        self.MLP.addConstrs(
            (self.C[s][jj] + self.u[s][jj] + self.big_M - self.big_M * self.x[j][jj][k] >= self.C[s][j] + self.u[s][
                j] for j in self.J for jj in self.J for s in range(3) for k in self.K_s[s + 1]), "con12")
        self.MLP.addConstrs(
            (self.C[s + 1][j] - self.o[s + 1][j] <= self.C[s][j] + self.u[s][j] + self.big_M - self.big_M *
             self.alpha[s + 1][j] for s in range(0, 3) for j in self.J[0:-2]), "con14")
        self.MLP.addConstrs(
            (self.C[s][j] - self.o[s][j] <= self.C[s][jj] + 2 * self.big_M - self.big_M * self.beta[s][
                j] - self.big_M * self.x[jj][j][k] for s in range(1, 4) for k in self.K_s[s] for j in self.J[0:-2]
             for jj in self.J), "con15")
        self.MLP.addConstrs((self.alpha[s][j] + self.beta[s][j] >= 1 for s in range(1, 4) for j in self.J[0:-2]),
                            "con16")
        self.MLP.addConstrs((self.o[s][-1] == 0 for s in self.S), "con202")
        self.MLP.addConstrs((self.o[s][-2] == 0 for s in self.S), "con203")
        self.MLP.addConstrs((self.u[s][self.J[-1]] == 0 for s in range(0, 3)), "con190")
        self.MLP.addConstrs((self.u[s][self.J[-2]] == 0 for s in range(0, 3)), "con191")
        tmp_mission_ls = deepcopy(self.env.mission_list)
        getattr(sort_missions, 'CHAR_ORDER')(tmp_mission_ls)
        self.MLP.addConstrs((self.v[j][k] == 1 for k in self.K for j in self.J_k[k]), "con8")
        for pair in self.A:
            j, jj = pair[0], pair[1]
            self.MLP.addConstr(self.r[jj] - self.C[0][j] >= 0, "con10" + str(j) + str(jj))
            k = int(tmp_mission_ls[j].quay_crane_id[-1]) - 1
            self.MLP.addConstr(self.x[j][jj][k] == 1, "con11" + str(j) + str(jj))
        self.MLP.addConstr(self.r[-2] == 0, "con00")
        self.MLP.addConstrs((self.o[s][j] == self.pt[s][j] for s in range(0, 3) for j in self.J[0:-2]), "con200")
        self.MLP.addConstrs((self.o[3][j] == self.pt[3][j] + sum(
            self.x[jj][j][k] * self.st[jj][j] for jj in self.J for k in self.K_s[3]) for j in self.J[0:-2]), "con201")
        self.MLP.addConstrs(
            (self.u[s][j] - self.tt[j][k][kk] + self.big_M * 2 - self.big_M * self.v[j][k] - self.big_M *
             self.v[j][kk] >= 0 for j in self.J[0:-2] for s in range(0, 3) for k in self.K_s[s] for kk in
             self.K_s[s + 1]), "con17")
        self.MLP.addConstrs(
            (self.u[s][j] - self.tt[j][k][kk] <= self.big_M * 2 - self.big_M * self.v[j][k] - self.big_M *
             self.v[j][kk] for j in self.J[0:-2] for s in range(0, 3) for k in self.K_s[s] for kk in self.K_s[s + 1]),
            "con18")
        q_1 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_1')
        q_2 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_2')
        self.MLP.addConstrs((q_1 >= self.C[3][j] for j in self.J), "obj1")
        self.MLP.addConstr((q_2 == sum(self.C[s + 1][j] - self.o[s + 1][j]
                                       - self.C[s][j] - self.u[s][j] for j in tmp_J for s in range(0, 3))), "obj2")
        self.MLP.addConstr(q_1 <= self.gamma, "obj1")
        # self.MLP.addConstr(q_1 >= self.gamma2, "obj1")
        self.MLP.setObjective(q_2, GRB.MINIMIZE)
        self.MLP.update()


def solve_model(MLP, inst_idx, solved_env: IterSolution = None, tag='', X_flag=True, Y_flag=True, epsilon=0.9):
    vars = MLP.getVars()
    machine_num = solved_env.iter_env.machine_num
    J_num_all = solved_env.iter_env.J_num_all
    if solved_env is not None:
        ls = solved_env.iter_env.mission_list
        if X_flag:
            for j in range(len(solved_env.iter_env.mission_list)):
                var_idx = (int(ls[j].idx[1:]) - 1) * machine_num + int(
                    ls[j].machine_list[4][-1]) + solved_env.iter_env.qc_num - 1
                MLP.addConstr((vars[var_idx] == 1), "fixed_x" + str(j))
        if Y_flag:
            for i in range(solved_env.iter_env.ls_num):
                ls_idx = solved_env.init_env.machine_name_to_idx['S' + str(i + 1)]
                getattr(sort_missions, "A_STATION_NB")(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list)
                for j in range(len(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list)):
                    p_mission_idx = int(
                        solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list[j].idx[1:]) - 1
                    if j == 0 and j == len(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list) - 1:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * J_num_all + \
                                  machine_num * p_mission_idx + ls_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_ls_" + str(i))
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * (J_num_all + 1) + ls_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_ls_" + str(i))
                    elif j == 0:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * J_num_all + \
                                  machine_num * p_mission_idx + ls_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_ls_" + str(i))
                        l_mission_idx = \
                            int(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * l_mission_idx + ls_idx
                        # Y[p_mission_idx][l_mission_idx][3 + i]
                        MLP.addConstr(vars[var_idx] == 1, "fixed_ls_" + str(i))
                    elif j == len(solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list) - 1:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * (J_num_all + 1) + ls_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_ls_" + str(i))
                    else:
                        l_mission_idx = int(
                            solved_env.iter_env.lock_stations['S' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * l_mission_idx + ls_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_ls_" + str(i))
            for i in range(len(solved_env.iter_env.crossovers)):
                co_idx = solved_env.init_env.machine_name_to_idx['CO' + str(i + 1)]
                getattr(sort_missions, "A_CROSSOVER_NB")(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list)
                for j in range(len(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list)):
                    p_mission_idx = int(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list[j].idx[1:]) - 1
                    if j == 0 and j == len(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list) - 1:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * J_num_all + \
                                  machine_num * p_mission_idx + co_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_co_" + str(i))
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * (J_num_all + 1) + co_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_co_" + str(i))
                    elif j == 0:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * J_num_all + \
                                  machine_num * p_mission_idx + co_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_co_" + str(i))
                        l_mission_idx = int(
                            solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * l_mission_idx + co_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_co_" + str(i))
                    elif j == len(solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list) - 1:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * (J_num_all + 1) + co_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_co_" + str(i))
                    else:
                        l_mission_idx = int(
                            solved_env.iter_env.crossovers['CO' + str(i + 1)].mission_list[j + 1].idx[1:]) - 1
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * l_mission_idx + co_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_co_" + str(i))
            for yc in solved_env.init_env.yard_cranes_set:
                yc_idx = solved_env.init_env.machine_name_to_idx['YC' + yc]
                getattr(sort_missions, "A_YARD_NB")(solved_env.iter_env.yard_cranes['YC' + yc].mission_list)
                for j in range(len(solved_env.iter_env.yard_cranes['YC' + yc].mission_list)):
                    p_mission_idx = int(solved_env.iter_env.yard_cranes['YC' + yc].mission_list[j].idx[1:]) - 1
                    if j == 0 and j == len(solved_env.iter_env.yard_cranes['YC' + yc].mission_list) - 1:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * J_num_all + \
                                  machine_num * p_mission_idx + yc_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_qc_" + yc)
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * (J_num_all + 1) + yc_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_qc_" + yc)
                    elif j == 0:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * J_num_all + \
                                  machine_num * p_mission_idx + yc_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_qc_" + yc)
                        l_mission_idx = int(solved_env.iter_env.yard_cranes['YC' + yc].mission_list[j + 1].idx[1:]) - 1
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * l_mission_idx + yc_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_qc_" + yc)
                    elif j == len(solved_env.iter_env.yard_cranes['YC' + yc].mission_list) - 1:
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * (J_num_all + 1) + yc_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_qc_" + yc)
                    else:
                        l_mission_idx = int(solved_env.iter_env.yard_cranes['YC' + yc].mission_list[j + 1].idx[1:]) - 1
                        var_idx = (J_num_all + 2) * machine_num + (J_num_all + 2) * machine_num * p_mission_idx + \
                                  machine_num * l_mission_idx + yc_idx
                        MLP.addConstr(vars[var_idx] == 1, "fixed_qc_" + yc)
    MLP.update()
    MLP.setParam('OutputFlag', 1)
    MLP.write("output_result/gurobi/mod_" + str(inst_idx) + "_" + tag + ".lp")
    # MLP.Params.timelimit = 7200
    T1 = time.time()
    MLP.optimize()
    # print("time: " + str(time.time() - T1))
    if MLP.status == GRB.Status.INFEASIBLE:
        print('Optimization was stopped with status %d' % MLP.status)
        # do IIS, find infeasible constraints
        MLP.computeIIS()
        for c in MLP.getConstrs():
            if c.IISConstr:
                print('%s' % c.constrName)
        MLP.write("output_result/gurobi/sol_" + str(inst_idx) + "_" + tag + ".ilp")
    elif MLP.status == GRB.Status.OPTIMAL:
        ls_ls = []
        m_ls = {}
        var_ls = {}
        for var in MLP.getVars():
            if int(var.x) != 0:
                tmp_str = var.VarName.split('_')
                if tmp_str[0] != 'x' and tmp_str[0] != 'v':
                    var_ls[var.VarName] = var.x
                if tmp_str[0] == 'v' and solved_env.iter_env.ls_num + solved_env.iter_env.qc_num - 1 >= int(
                        tmp_str[2]) >= solved_env.iter_env.qc_num and int(tmp_str[1]) < J_num_all:
                    ls_ls.append(int(tmp_str[2]) - solved_env.iter_env.qc_num)
                if tmp_str[0] == 'x':
                    if m_ls.get(int(tmp_str[-1])) is None:
                        m_ls.setdefault(int(tmp_str[-1]), [int(tmp_str[1]), int(tmp_str[2]), -1])
                    else:
                        tmp: list = m_ls.get(int(tmp_str[-1]))
                        if tmp.count(int(tmp_str[1])) != 0:
                            tmp.insert(tmp.index(int(tmp_str[1])) + 1, int(tmp_str[2]))
                        elif tmp.count(int(tmp_str[2])) != 0:
                            tmp.insert(tmp.index(int(tmp_str[2])), int(tmp_str[1]))
                        else:
                            tmp.extend([int(tmp_str[1]), int(tmp_str[2])])
        m_ls = sorted(m_ls.items(), key=lambda d: d[0], reverse=False)
        var_ls = [solved_env.iter_env.qc_num + i * machine_num + (J_num_all + 2) * machine_num
                  for i in range(0, (J_num_all + 2) * (J_num_all + 2))]
        var_ls.extend([solved_env.iter_env.qc_num + 1 + i * machine_num + (J_num_all + 2) * machine_num
                       for i in range(0, (J_num_all + 2) * (J_num_all + 2))])
        var_ls.extend([solved_env.iter_env.qc_num + 2 + i * machine_num + (J_num_all + 2) * machine_num
                       for i in range(0, (J_num_all + 2) * (J_num_all + 2))])
        var_ls.extend([solved_env.iter_env.qc_num + 3 + i * machine_num + (J_num_all + 2) * machine_num
                       for i in range(0, (J_num_all + 2) * (J_num_all + 2))])
        f = open("output_result/RL.txt", "a")
        f.write('Makespan: ' + str(MLP.getVars()[-2].x))
        f.write('Congestion time:' + str(MLP.getVars()[-1].x))
        f.close()
    else:
        print("NO optimal solution" + str(MLP.status))
    return MLP


def solve_model_fixed(MLP, inst_idx):
    MLP.update()
    MLP.setParam('OutputFlag', 1)
    T1 = time.time()
    MLP.optimize()
    if MLP.status == GRB.Status.INFEASIBLE:
        print('Optimization was stopped with status %d' % MLP.status)
        # do IIS, find infeasible constraints
        MLP.computeIIS()
        for c in MLP.getConstrs():
            if c.IISConstr:
                print('%s' % c.constrName)
        MLP.write("output_result/gurobi/sol_" + str(inst_idx) + ".ilp")
    elif MLP.status == GRB.Status.OPTIMAL:
        f = open("output_result/RL.txt", "a")
        f.write('Makespan: ' + str(MLP.getVars()[-2].x))
        f.write('Congestion time:' + str(MLP.getVars()[-1].x))
        f.close()
    else:
        print("NO optimal solution" + str(MLP.status))
    return MLP
