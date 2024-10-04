#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：buffer.py
@Author  ：JacQ
@Date    ：2021/12/21 9:24
"""
from common.machine import Machine


class Buffer(Machine):
    def __init__(self, buffer_id, location, handling_time, capacity):
        super(Buffer, self).__init__(buffer_id, location)
        self.handling_time = handling_time
        self.capacity = capacity
