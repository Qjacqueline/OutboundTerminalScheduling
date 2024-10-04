import time
from copy import deepcopy

from gurobipy import *
from algorithm_factory.algo_utils import sort_missions
from common.iter_solution import IterSolution
import conf.configs as cf
from data_process.input_process import read_input, generate_data_for_test


class SeparateModel:
    def __init__(self, solu: IterSolution, stage, pre_result):
        self.env = solu.init_env
        self.solu = solu
        self.S_num = 4
        self.J_num = self.env.J_num
        self.J_num_all = self.env.J_num_all
        self.K_num = -1
        self.big_M = 10000
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

        self.MLP = None
        self.v = None  # v_jk
        self.x = None  # x_ijk
        self.C = None  # C^s_j
        self.o = None  # o^s_j
        self.r = None  # r_j
        self.u = None  # u^s_j
        self.alpha = None
        self.beta = None

        self.stage = stage
        self.res = pre_result

        self.init_model()

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

        if self.stage == 2:
            self.MLP.addConstrs((self.C[0][job_num + 1] <= self.res[0][job_num] for job_num in range(self.J_num_all)),
                                "I1")
        if self.stage == 3:
            self.MLP.addConstrs((self.C[0][job_num + 1] <= self.res[0][job_num] for job_num in range(self.J_num_all)),
                                "I1")
            self.MLP.addConstrs((self.C[1][job_num + 1] <= self.res[1][job_num] for job_num in range(self.J_num_all)),
                                "I2")
        if self.stage == 4:
            self.MLP.addConstrs((self.C[0][job_num + 1] <= self.res[0][job_num] for job_num in range(self.J_num_all)),
                                "I1")
            self.MLP.addConstrs((self.C[1][job_num + 1] <= self.res[1][job_num] for job_num in range(self.J_num_all)),
                                "I2")
            self.MLP.addConstrs((self.C[2][job_num + 1] <= self.res[2][job_num] for job_num in range(self.J_num_all)),
                                "I3")

        # ============== 构造目标 ================
        q_1 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_1')
        q_2 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_2')
        q_3 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_3')
        q_4 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_4')
        q_5 = self.MLP.addVar(lb=0, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name='q_5')
        self.MLP.addConstrs((q_1 >= self.C[0][j] for j in self.J), "obj1")
        self.MLP.addConstrs((q_2 >= self.C[1][j] for j in self.J), "obj1")
        self.MLP.addConstrs((q_3 >= self.C[2][j] for j in self.J), "obj1")
        self.MLP.addConstrs((q_4 >= self.C[3][j] for j in self.J), "obj1")
        if self.stage == 1:
            self.MLP.setObjective(q_1, GRB.MINIMIZE)
        elif self.stage == 2:
            self.MLP.setObjective(q_2, GRB.MINIMIZE)
        elif self.stage == 3:
            self.MLP.setObjective(q_3, GRB.MINIMIZE)
        else:
            self.MLP.setObjective(q_4, GRB.MINIMIZE)
        self.MLP.update()
        self.MLP.setParam('OutputFlag', 0)
        self.MLP.Params.timelimit = 7200
        self.MLP.optimize()
        if self.stage == 1:
            self.res.append(
                [self.MLP.getVarByName('C_' + '0_' + str(job_num + 1)).X for job_num in range(self.J_num_all)])
        if self.stage == 2:
            self.res.append(
                [self.MLP.getVarByName('C_' + '1_' + str(job_num + 1)).X for job_num in range(self.J_num_all)])
        if self.stage == 3:
            self.res.append(
                [self.MLP.getVarByName('C_' + '2_' + str(job_num + 1)).X for job_num in range(self.J_num_all)])


def run(solu):
    f = open("output_result\\separate.txt", "a")
    # Separate
    # try:
    model1 = SeparateModel(solu, 1, [])
    model2 = SeparateModel(solu, 2, model1.res)
    model3 = SeparateModel(solu, 3, model2.res)
    model4 = SeparateModel(solu, 4, model3.res)
    f.write("sep\t" + str(model1.MLP.ObjVal) + "\t" + str(model2.MLP.ObjVal) + "\t" + str(
        model3.MLP.ObjVal) + "\t" + str(model4.MLP.ObjVal) + "\t")
    # except:
    #     print("wrong")
    # Integrate
    model5 = SeparateModel(solu, 5, [])
    f.write("int\t" + str(model5.MLP.ObjVal) + "\n")


if __name__ == '__main__':
    # m_num_ls = [10, 10, 10,
    #             10, 10, 10][10, 10, 10, 10, 10,
    # inst_type_ls = ['A2_t', 'A2_t', 'B2_t', 'B2_t', 'C2_t', 'C2_t',
    #                 'G2_t', 'G2_t', 'H2_t', 'H2_t', 'Z2_t', 'Z2_t']'A2_t', 'B2_t', 'C2_t', 'G2_t', 'H2_t',
    m_num_ls = [10, 10, 10, 10, 10, 10, 12, 15, 11, 17, 14, 16]
    inst_type_ls = ['A2_t', 'B2_t', 'C2_t', 'G2_t', 'H2_t', 'Z2_t', 'A2_t', 'B2_t', 'C2_t', 'G2_t', 'H2_t', 'Z2_t']
    f = open("output_result\\separate.txt", "a")
    for i in range(len(m_num_ls)):
        for j in range(10):
            print(str(j))
            cnt = 0
            flag = True
            while True:
                env = generate_data_for_test(j, inst_type=inst_type_ls[i], mission_num=m_num_ls[i])
                cnt += 1
                if len(env.yard_cranes_set) == env.yc_num:
                    break
                if cnt > 20:
                    flag = False
                    print("generate error")
                    break
            if flag:
                solu = read_input(pre='integrate', inst_idx=j, inst_type=inst_type_ls[i], mission_num=m_num_ls[i])
                solu.l2a_init()
                try:
                    run(solu)
                except:
                    print(inst_type_ls[i] + str(m_num_ls[i]) + " unsolved")
            else:
                print(inst_type_ls[i] + str(m_num_ls[i]) + " fail")
        f.write("\n")
