#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：Port_Scheduling_New_Version
@File    ：upper_agent_new.py
@Author  ：JacQ
@Date    ：2022/5/11 17:31
"""

from common.mission import Mission

from abc import ABC, abstractmethod
from typing import Callable, List

import torch
import torch.nn.functional as F
from tensorboardX import SummaryWriter
from torch import nn
from tqdm import tqdm
import conf.configs as cf
from torch_geometric.loader import DataLoader
from algorithm_factory.algo_utils.data_buffer import UANewBuffer
from algorithm_factory.algo_utils.machine_cal_methods import get_state, get_next_job_at_quay_cranes, get_state_n
from algorithm_factory.algo_utils.net_models import ActorNew, CriticNew
from algorithm_factory.algo_utils.rl_methods import print_result
from algorithm_factory.algo_utils.rl_methods import soft_update
from algorithm_factory.rl_algo.lower_agent import DDQN
from common.iter_solution import IterSolution
from utils.log import Logger

logger = Logger().get_logger()


class BaseAgent(ABC, nn.Module):
    def __init__(self):
        super(BaseAgent, self).__init__()
        """
        """

    @abstractmethod
    def forward(self, **kwargs):
        """
        """

    @abstractmethod
    def update(self, **kwargs):
        """

        """

    def sync_weight(self) -> None:
        """        Soft-update the weight for the target network.        """
        soft_update(self.target_net, self.eval_net, self.soft_target_tau)


class ACUpper(BaseAgent):
    def __init__(self,
                 actor: ActorNew,
                 critic: CriticNew,
                 actor_lr: float,
                 critic_lr: float,
                 gamma: float,
                 device: torch.device,
                 loss_fn: Callable = nn.MSELoss(),
                 ) -> None:
        super(ACUpper, self).__init__()
        logger.info("CREATE DDPG u_agent")
        self.actor = actor.to(device)
        self.critic = critic.to(device)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=critic_lr)
        # self.optimizer = torch.optim.Adam(self.qf.parameters(), lr=1e-4)
        self.gamma = gamma
        self.device = device
        self.loss_func = loss_fn

        self.noise_lim = torch.tensor(0.3)
        self.noise_std = torch.tensor(0.1)

    def forward(self, state, eval_tag=True):
        action = self.actor.forward(state)
        if eval_tag:
            action = action
            return action.detach().cpu()
        else:
            noise = (self.noise_std * torch.randn_like(action)).clamp(-self.noise_lim, self.noise_lim).to(self.device)
            action = (action + noise).clamp(10, 90)
            action = action
            return action.detach().cpu()

    def update(self, batch: List):
        s = batch[0].to(self.device)
        l_action = batch[1].unsqueeze(1).to(self.device)
        s_ = batch[2].to(self.device)
        rewards = batch[3].unsqueeze(1).to(self.device)
        done = batch[4].unsqueeze(1).to(self.device)
        u_action = batch[5].unsqueeze(1).to(self.device)

        u_a = self.actor.forward(s)
        n_u_a = self.actor.forward(s_)

        # Compute the target Q value
        target_Q = self.critic(s_, n_u_a, l_action)
        target_Q = rewards + (done * self.gamma * target_Q).detach()

        # Get current Q estimate
        current_Q = self.critic(s, u_action, l_action)

        # Compute critic loss
        critic_loss = F.mse_loss(current_Q, target_Q)

        # Optimize the critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Compute actor loss
        actor_loss = -self.critic(s, u_a, l_action).mean()
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        return actor_loss, critic_loss, u_action.mean()


class UANewCollector:
    def __init__(self, train_solus: List[IterSolution], test_solus: List[IterSolution], mission_num: int,
                 m_max_num: int, each_quay_m_num: int, data_buffer: UANewBuffer, batch_size: int, u_agent: ACUpper,
                 l_agent: DDQN, rl_logger: SummaryWriter, save_path: str, init_gap=cf.RELEASE_TIME):
        logger.info("CREATE data Collector")
        self.train_solus = train_solus
        self.test_solus = test_solus
        self.mission_num = mission_num
        self.m_max_num = m_max_num
        self.each_quay_m_num = each_quay_m_num
        self.data_buffer = data_buffer
        self.batch_size = batch_size
        self.u_dl_train = None
        self.u_agent = u_agent
        self.l_agent = l_agent
        self.best_result = [float('Inf') for _ in range(len(self.test_solus) + 2)]
        self.rl_logger = rl_logger
        self.save_path = save_path
        self.pre_makespan = 0
        self.curr_time = [0, 0, 0]
        self.init_release_time_gap = init_gap
        self.train_time = 0
        self.task = cf.inst_type + '_' + str(cf.MISSION_NUM_ONE_QUAY_CRANE)

    def collect_rl(self):
        for solu in self.train_solus:
            self.curr_time = [0, 0, 0]
            pre_makespan = 0
            state = get_state_n(env=solu.iter_env, step_number=0, max_num=self.m_max_num)
            for step in range(self.mission_num):
                cur_mission = get_next_job_at_quay_cranes(solu.iter_env, self.curr_time)
                solu.released_missions.append(cur_mission)
                u_action = self.u_agent.forward(state)
                l_action = self.l_agent.forward(state, False)
                makespan = self.process(solu, cur_mission, u_action, l_action, step)
                reward = (pre_makespan - makespan)
                if step != self.mission_num - 1:
                    new_state = get_state_n(env=solu.iter_env, step_number=step + 1, max_num=self.m_max_num)
                    done = 0
                else:
                    new_state = state
                    done = 1
                # record & train & eval
                if step >= 3:
                    self.data_buffer.append(state=state, u_ac=float(u_action), l_ac=l_action,
                                            new_state=new_state, r=reward, done=done)
                if self.train_time == 3:
                    self.u_dl_train = DataLoader(dataset=self.data_buffer, batch_size=self.batch_size, shuffle=True)
                if self.train_time > 3 and self.train_time % 2 == 0:
                    self.train()

                # update
                state = new_state
                pre_makespan = makespan
                self.train_time = self.train_time + 1
            solu.reset()

    def eval(self, l_eval_flag=False):
        with torch.no_grad():
            makespan_forall = []
            reward_forall = []
            for i in range(len(self.test_solus)):
                torch.manual_seed(42)
                solu = self.test_solus[i]
                state = get_state_n(env=solu.iter_env, step_number=0, max_num=self.m_max_num)
                pre_makespan = 0
                total_reward = 0
                self.curr_time = [0, 0, 0]
                # print(i)
                for step in range(self.mission_num):
                    cur_mission = get_next_job_at_quay_cranes(solu.iter_env, self.curr_time)
                    solu.released_missions.append(cur_mission)
                    if l_eval_flag:
                        u_action = torch.tensor(self.init_release_time_gap)
                    else:
                        u_action = self.u_agent.forward(state, False)
                    l_action = self.l_agent.forward(state, False)
                    makespan = self.process(solu, cur_mission, u_action, l_action, step)
                    total_reward += (pre_makespan - makespan)
                    if step != self.mission_num - 1:
                        new_state = get_state_n(env=solu.iter_env, step_number=step + 1,
                                                max_num=self.m_max_num)
                    else:
                        new_state = state
                    pre_makespan = makespan
                    state = new_state
                solu.reset()
                makespan_forall.append(makespan)
                reward_forall.append(total_reward)
                if makespan < self.best_result[i]:
                    self.best_result[i] = solu.last_step_makespan
            makespan_forall.append(sum(makespan_forall[0:len(self.train_solus)]))
            makespan_forall.append(sum(makespan_forall[0:-1]) - sum(makespan_forall[0:len(self.train_solus)]))
            reward_forall.append(sum(reward_forall[0:len(self.train_solus)]))
            if makespan_forall[-2] < self.best_result[-2]:
                self.best_result[-2] = makespan_forall[-2]
            if makespan_forall[-1] < self.best_result[-1]:
                self.best_result[-1] = makespan_forall[-1]
                torch.save(self.l_agent.qf, self.save_path + '/eval_' + self.task + 'l.pkl')
                torch.save(self.l_agent.qf_target, self.save_path + '/target_' + self.task + 'l.pkl')
                torch.save(self.u_agent.actor, self.save_path + '/actor_' + self.task + 'l.pkl')
                torch.save(self.u_agent.critic, self.save_path + '/critic_' + self.task + 'l.pkl')
            return makespan_forall, reward_forall

    def train(self, train_num=3):
        total_policy_loss = 0
        total_vf_loss = 0
        total_loss = 0
        total_q_eval = 0
        total_q_eval_value = 0
        train_batch_num = 0

        # train upper
        batch = next(iter(self.u_dl_train))
        policy_loss, vf_loss, u_action_mean = self.u_agent.update(batch)
        total_policy_loss += policy_loss.data
        total_vf_loss += vf_loss.data

        # train lower
        for i in range(train_num):
            batch = next(iter(self.u_dl_train))
            loss, q_eval, q_eval_value = self.l_agent.update(batch)
            total_loss += loss.data
            total_q_eval += q_eval.data
            total_q_eval_value += q_eval_value.data
            train_batch_num += 1
        self.rl_logger.add_scalar(tag=f'u_train/policy_loss', scalar_value=total_policy_loss,
                                  global_step=self.train_time)
        self.rl_logger.add_scalar(tag=f'u_train/vf_loss', scalar_value=total_vf_loss,
                                  global_step=self.train_time)
        self.rl_logger.add_scalar(tag=f'u_train/u_action_mean', scalar_value=u_action_mean,
                                  global_step=self.train_time)
        if self.train_time % 20 == 0:
            makespan, reward = self.eval()
            # field_name = ['Epoch', 'policy_loss', 'vf_loss', 'q_loss']
            # value = [self.train_time, total_policy_loss, total_vf_loss, total_loss]
            field_name = ['Epoch', 'policy_loss', 'vf_loss', 'u_action_mean', 'makespan_train', 'makespan_test',
                          'q_loss']
            value = [self.train_time, total_policy_loss, total_vf_loss, u_action_mean, makespan[-2], makespan[-1],
                     torch.sqrt(total_loss) * train_num / total_q_eval]
            self.rl_logger.add_scalar(tag=f'l_train/makespan_train', scalar_value=makespan[-2],
                                      global_step=self.train_time)
            self.rl_logger.add_scalar(tag=f'l_train/makespan_test', scalar_value=makespan[-1],
                                      global_step=self.train_time)
            # for i in range(len(makespan)):
            #     self.rl_logger.add_scalar(tag=f'l_train/makespan' + str(i + 1), scalar_value=makespan[i],
            #                               global_step=self.train_time)
            #     field_name.append('makespan' + str(i + 1))
            #     value.append(makespan[i])
            print_result(field_name=field_name, value=value)

    def process(self, solu: IterSolution, mission: Mission, u_action: torch.Tensor, l_action: torch.Tensor, step: int):
        self.process_release_adjust(mission, self.each_quay_m_num, u_action)
        cur_makespan = solu.step_v22(action=l_action, mission=mission, step_number=step)
        return cur_makespan

    def process_release_adjust(self, mission: Mission, each_quay_m_num, u_action=torch.tensor(0)):
        adjust_release_time = mission.release_time
        if int(mission.idx[1:]) % each_quay_m_num == 1:
            mission.release_time = 0
            adjust_release_time = 0
            self.curr_time[int(int(mission.idx[1:]) / each_quay_m_num)] = self.init_release_time_gap
        elif int(mission.idx[1:]) <= each_quay_m_num:
            adjust_release_time = self.curr_time[0] + u_action.item() - self.init_release_time_gap
            self.curr_time[0] = adjust_release_time + self.init_release_time_gap
        elif each_quay_m_num < int(mission.idx[1:]) <= each_quay_m_num * 2:
            adjust_release_time = self.curr_time[1] + u_action.item() - self.init_release_time_gap
            self.curr_time[1] = adjust_release_time + self.init_release_time_gap
        elif each_quay_m_num * 2 < int(mission.idx[1:]) <= each_quay_m_num * 3:
            adjust_release_time = self.curr_time[2] + u_action.item() - self.init_release_time_gap
            self.curr_time[2] = adjust_release_time + self.init_release_time_gap
        transfer_time = mission.machine_start_time[1] - mission.release_time
        mission.machine_start_time[0] = adjust_release_time
        mission.machine_start_time[1] = adjust_release_time + transfer_time
        mission.release_time = adjust_release_time

