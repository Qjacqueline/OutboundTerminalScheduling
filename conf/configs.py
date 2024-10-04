#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：port_scheduling
@File    ：configs.py
@Author  ：JacQ
@Date    ：2021/12/1 16:56
"""
import logging
import os
import numpy as np
import torch

LOGGING_LEVEL = logging.INFO  # logging.WARNING/DEBUG
inst_type = 'I2_t'
MISSION_NUM = 5000
STAGE_NUM = 3
QUAY_EXIT = np.array([280, 0])
QUAYCRANE_EXIT = -60
QUAYCRANE_CRANE = -45
S1_STATION_LOCATION = np.array([280 + 2.6 * 25, 1.5 * 25])
LOCK_STATION_SPACE = -4.3 * 25
LOCK_STATION_BUFFER_SPACE = 5
SLOT_LENGTH = 5.89
SLOT_WIDTH = 2.44
SLOT_NUM_X = 30
SLOT_NUM_Y = 12
LANE_X = 5.5 * 25
LANE_Y = 1.2 * 25
LANE_WIDTH = 2 * SLOT_WIDTH
BLOCK_SPACE_X = SLOT_LENGTH * SLOT_NUM_X + LANE_X
BLOCK_SPACE_Y = SLOT_WIDTH * SLOT_NUM_Y + LANE_Y
A1_LOCATION = np.array([280 - 13 * 25, 4 * 25]) + np.array([-SLOT_LENGTH * SLOT_NUM_X, SLOT_WIDTH * SLOT_NUM_Y])
BLOCK_IS_SPACE_X = 2 * 25
BLOCK_IS_SPACE_Y = 0.4 * 25
RELEASE_TIME = 120
QUAYCRANE_PROCESS_TIME = [108, 130]
BUFFER_PROCESS_TIME = 60
LOCK_STATION_HANDLING_TIME = [40, 90]
WAIT_TIME_DELAY = [0, 0, 0, 0]
CROSSOVER_HANDLING_TIME = [20, 60]
YARDCRANE_SPEED_X = 2.17
YARDCRANE_SPEED_Y = 1.8
MAX_MOVE_TIME = BLOCK_SPACE_X / YARDCRANE_SPEED_X + BLOCK_SPACE_Y / YARDCRANE_SPEED_Y
YARDCRANE_HANDLING_TIME = [25, 35]
VEHICLE_SPEED = [6, 7]
RANDOM_SEED = 10
INITIAL_EPSILON = 0.8
RL_CONFIG = 6
DEVICE = torch.device("cuda:1" if torch.cuda.is_available() else 'cpu')
ROOT_FOLDER_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_PATH = os.path.join(ROOT_FOLDER_PATH, 'data/data_' + inst_type)
if not os.path.exists(DATA_PATH):
    os.makedirs(DATA_PATH)
OUTPUT_RESULT_PATH = os.path.join(ROOT_FOLDER_PATH, 'output/output_result')
OUTPUT_PATH = os.path.join(OUTPUT_RESULT_PATH, 'output_' + str(inst_type) + '.json')
LAYOUT_PATH = os.path.join(OUTPUT_RESULT_PATH, 'layout.png')
MODEL_PATH = os.path.join(OUTPUT_RESULT_PATH, 'model')
LOSS_PLOT_PATH = os.path.join(OUTPUT_RESULT_PATH, 'loss_')
OUTPUT_SOLUTION_PATH = os.path.join(OUTPUT_RESULT_PATH, 'solution_' + inst_type + '/')
T0 = 4000
TEND = 1e-10
RATE = 0.995
