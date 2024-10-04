#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@File    ：mission.py
@Author  ：JacQ
@Date    ：2021/12/1 16:53
"""


class Mission(object):

    def __init__(self, idx, quay_crane_id, yard_block_loc, yard_crane_process_time, intersection_process_time, locked,
                 release_time, vehicle_speed, station_process_time):
        """
        Initializes the Mission class with all the relevant attributes for a port scheduling task.

        :param idx: Task ID
        :param quay_crane_id: Quay crane ID
        :param yard_block_loc: The location of the storage yard ('A1', 5, 6), where A1 is the yard, x=5 and y=6 is the slot
        :param yard_crane_process_time: The time required for the yard crane to complete its task
        :param intersection_process_time: Time required at intersections during the task
        :param locked: Indicates whether the task is locked
        :param release_time: The time when the task is released (task start time)
        :param vehicle_speed: Speed of the transport vehicle
        :param station_process_time: The time required to process the task at a station
        """

        self.idx = idx
        self.locked = locked
        self.vehicle_speed = vehicle_speed
        self.quay_crane_id = quay_crane_id
        self.release_time = release_time
        self.station_process_time = station_process_time
        self.intersection_process_time = intersection_process_time
        self.yard_crane_process_time = yard_crane_process_time
        self.crossover_id = None  # ID for crossover locations
        self.yard_block_loc = yard_block_loc  # Location in the yard
        self.yard_stop_loc = []  # Stops in the yard during the task

        # Transfer times between various points in the task
        self.transfer_time_e2s_min = 0  # Transfer time from entry to station (minimum)
        self.transfer_time_s2c_min = 0  # Transfer time from station to crossover (minimum)
        self.transfer_time_c2y = 0  # Transfer time from crossover to yard

        self.total_process_time = 0  # Total processing time for the task
        self.machine_list = []  # List of machines assigned to each stage
        self.machine_start_time = []  # Start time for each machine operation
        self.machine_process_time = []  # Processing time for each machine operation

        self.waiting_time = []  # Time spent waiting at each stage
        self.process_time = []  # Time spent processing at each stage
        self.arriving_time = []  # Arrival times at each stage
        self.stage = 1  # Initial stage of the task

    def cal_mission_attributes(self, buffer_flag=True):
        # Calculates the mission attributes, such as waiting time, process time, and arrival time,
        # based on whether buffer stages are included or not (station-crossover-yard-station_buffer)
        if buffer_flag:
            # With buffer stages
            self.waiting_time = [self.machine_process_time[2], self.machine_process_time[5],
                                 self.machine_process_time[7], self.machine_process_time[3]]
            self.process_time = [self.machine_process_time[4], self.machine_process_time[6],
                                 self.machine_process_time[8],
                                 self.machine_process_time[2] - self.machine_process_time[3]]
            self.arriving_time = [self.machine_start_time[2], self.machine_start_time[5],
                                  self.machine_start_time[7], self.machine_start_time[2]]

        else:
            # Without buffer stages
            self.waiting_time = [self.machine_process_time[2], self.machine_process_time[4],
                                 self.machine_process_time[6]]
            self.process_time = [self.machine_process_time[3], self.machine_process_time[5],
                                 self.machine_process_time[7]]
            self.arriving_time = [self.machine_start_time[2], self.machine_start_time[4],
                                  self.machine_start_time[6]]
