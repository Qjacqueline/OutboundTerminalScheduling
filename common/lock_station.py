#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：lock_station.py
@Author  ：JacQ
@Date    ：2021/12/1 16:54
"""
from common.machine import Machine


class LockStation(Machine):
    def __init__(self, lock_station_id, location, capacity, lock_station_buffers):
        super(LockStation, self).__init__(lock_station_id, location)
        self.capacity = capacity
        self.lock_station_buffers = lock_station_buffers
        self.whole_occupy_time = []
        self.distance_to_exit = 0
