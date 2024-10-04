#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：port_scheduling
@File    ：yard_crane.py
@Author  ：JacQ
@Date    ：2021/12/1 16:55
"""
from common.machine import Machine


class YardCrane(Machine):
    def __init__(self, yard_crane_id, block_id, location, x_move_rate, y_move_rate, handling_time):
        super(YardCrane, self).__init__(yard_crane_id, location)
        self.block_id = block_id
        self.x_move_rate = x_move_rate
        self.y_move_rate = y_move_rate
        self.handling_time = handling_time
