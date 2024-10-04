#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：port_scheduling
@File    ：yard_block.py
@Author  ：JacQ
@Date    ：2021/12/20 17:47
"""


class YardBlock(object):
    def __init__(self, yard_block_id, block_location):
        self.idx = yard_block_id
        self.block_location = block_location
        self.slot_width = 0
        self.slot_length = 0
        self.lane_loc = []
