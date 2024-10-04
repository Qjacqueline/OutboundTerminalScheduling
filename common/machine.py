#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：Machine.py
@Author  ：JacQ
@Date    ：2022/1/4 16:29
"""


class Machine:
    def __init__(self, idx, location):
        self.idx = idx
        self.location = location
        self.mission_list = []
        self.process_time = []
