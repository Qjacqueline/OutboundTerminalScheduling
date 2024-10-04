#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：port_scheduling
@File    ：quay_crane.py
@Author  ：JacQ
@Date    ：2021/12/1 16:55
"""
from common.machine import Machine


class QuayCrane(Machine):
    def __init__(self, quay_crane_id, missions, location):
        super(QuayCrane, self).__init__(quay_crane_id, location)
        self.missions = missions  # tasks dispatch by TOS
        self.time_to_exit = 0
