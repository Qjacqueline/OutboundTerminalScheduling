#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：lock_station_buffer.py
@Author  ：JacQ
@Date    ：2022/2/17 10:21
"""

from common.machine import Machine


class LockStationBuffer(Machine):
    def __init__(self, lock_station_buffer_id, location, lock_station_idx, wait_time_delay):
        super(LockStationBuffer, self).__init__(lock_station_buffer_id, location)
        self.lock_station_idx = lock_station_idx
        self.wait_time_delay = wait_time_delay
