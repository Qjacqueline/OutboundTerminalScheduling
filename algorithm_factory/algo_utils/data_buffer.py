#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：u_data_buffer.py
@Author  ：JacQ
@Date    ：2022/4/21 10:28
"""
from abc import ABC
from typing import Tuple, Any, List

import torch

from torch_geometric.data import Dataset

from utils.log import Logger

logger = Logger().get_logger()


class LABuffer(Dataset, ABC):
    def __init__(self, buffer_size: int = 30000) -> None:
        super(LABuffer, self).__init__()
        self.state = []
        self.action = []
        self.state_ = []
        self.reward = []
        self.done = []
        self.buffer_size = buffer_size

    def __len__(self) -> int:
        return len(self.action)

    def __getitem__(self, index: int) -> Tuple:
        return (self.state[index],
                self.action[index],
                self.state_[index],
                self.reward[index],
                self.done[index]
                )

    def append(self, state, ac, new_state, r, done):
        if len(self.done) == self.buffer_size:
            self.pop()
        r = r / 100.0
        self.state.append(state)
        self.action.append(ac)
        self.state_.append(new_state)
        self.reward.append(r)
        self.done.append(done)

    def pop(self):
        self.state.pop(0)
        self.action.pop(0)
        self.state_.pop(0)
        self.reward.pop(0)
        self.done.pop(0)


def la_collate_fn(raw_batch: List) -> Any:
    zip_raw_batch = zip(*raw_batch)
    return {'state': next(zip_raw_batch),
            'l_action': torch.tensor(next(zip_raw_batch), dtype=torch.int64),
            'state_': next(zip_raw_batch),
            'reward': torch.tensor(next(zip_raw_batch), dtype=torch.float32),
            'done': torch.tensor(next(zip_raw_batch), dtype=torch.float32)
            }


class UABuffer(Dataset, ABC):
    def __init__(self, buffer_size: int = 30000) -> None:
        super(UABuffer, self).__init__()
        logger.info("data buffer")
        self.state = []
        self.adjust = []
        self.action = []
        self.state_ = []
        self.reward = []
        self.buffer_size = buffer_size

    def __len__(self) -> int:
        return len(self.action)

    def __getitem__(self, index: int) -> Tuple:
        return (self.state[index],
                self.adjust[index],
                self.action[index],
                self.state_[index],
                self.reward[index]
                )

    def append(self, state, ac, r, new_state, adjust):
        if len(self.action) == self.buffer_size:
            self.pop()
        r = r / 100.0
        self.state.append(state)
        self.adjust.append(adjust)
        self.action.append(ac)
        self.state_.append(new_state)
        self.reward.append(r)

    def pop(self):
        self.state.pop(0)
        self.adjust.pop(0)
        self.action.pop(0)
        self.state_.pop(0)
        self.reward.pop(0)

    def clear(self):
        self.state = []
        self.adjust = []
        self.action = []
        self.state_ = []
        self.reward = []


class UANewBuffer(Dataset, ABC):
    def __init__(self, buffer_size: int = 30000) -> None:
        super(UANewBuffer, self).__init__()
        logger.info("data buffer")
        self.state = []

        self.u_action = []
        self.l_action = []

        self.state_ = []

        self.reward = []
        self.done = []
        self.buffer_size = buffer_size

    def __len__(self) -> int:
        return len(self.l_action)

    def __getitem__(self, index: int) -> Tuple:
        return (self.state[index],
                self.l_action[index],
                self.state_[index],
                self.reward[index],
                self.done[index],
                self.u_action[index]
                )

    def append(self, state, u_ac, l_ac, new_state, r, done):
        if len(self.done) == self.buffer_size:
            self.pop()
        self.state.append(state)
        self.u_action.append(u_ac)
        self.l_action.append(l_ac)
        self.state_.append(new_state)
        self.reward.append(r)
        self.done.append(done)

    def pop(self):
        self.state.pop(0)
        self.u_action.pop(0)
        self.l_action.pop(0)
        self.state_.pop(0)
        self.reward.pop(0)
        self.done.pop(0)

    def clear(self):
        self.state = []
        self.u_action = []
        self.l_action = []
        self.state_ = []
        self.reward = []
        self.done = []
