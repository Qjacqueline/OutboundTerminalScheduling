#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：port_scheduling
@File    ：plot_layout.py
@Author  ：JacQ
@Date    ：2021/12/22 11:22
"""
import matplotlib.pyplot as plt
import numpy as np

import conf.configs as Cf
from input_process import read_input


def plot_layout(port_env):
    plt.figure()
    plt.xticks(np.arange(0, 900, 100))
    plt.yticks(np.arange(-30, 300, 100))
    # plt.axis('off')
    plt.axhline(y=0, xmin=0, xmax=1, ls='--', linewidth=0.5, c="k")
    for qc_id, quay_crane in port_env.quay_cranes.items():
        qc_loc = quay_crane.location
        plt.gca().add_patch(plt.Rectangle((qc_loc[0] + 5, -30), 10, 30, facecolor='lightgrey'))
        plt.text(qc_loc[0] - 7, -12, qc_id, fontsize=6)
        plt.gca().add_patch(plt.Rectangle((qc_loc[0], 0), 20, 5, facecolor='grey'))
        plt.text(qc_loc[0] - 7, 9, 'BF' + qc_id[-1], fontsize=5.5)
        plt.text(qc_loc[0] - 15, 5, str(qc_loc), fontsize=5, color='r')
    for station_id, station in port_env.lock_stations.items():
        plt.gca().add_patch(plt.Rectangle((station.location[0], station.location[1]), 20, 2, color='blue', alpha=0.5))
        plt.text(station.location[0], station.location[1], station_id, fontsize=5)
        plt.text(station.location[0], station.location[1] - 5, str(station.location),
                 fontsize=5, color='r')
        # for station_buffer_id, station_buffer in station.lock_station_buffers.items():
        #     plt.gca().add_patch(
        #         plt.Rectangle((station_buffer.location[0], station_buffer.location[1]), 20, 2, color='blue', alpha=0.1))
        #     plt.text(station_buffer.location[0], station_buffer.location[1], station_buffer_id, fontsize=3)
    for co_id, co in port_env.crossovers.items():
        # plt.gca().add_patch(plt.Rectangle((co.location[0], co.location[1]), 20, 2, color='blue', alpha=0.5))
        plt.text(co.location[0], co.location[1], co_id, fontsize=5)
        plt.text(co.location[0], co.location[1] - 5, str(co.location),
                 fontsize=5, color='r')
    for block_id, block in port_env.yard_blocks.items():
        plt.gca().add_patch(
            plt.Rectangle((block.block_location[0], block.block_location[1]), Cf.SLOT_NUM_X * Cf.SLOT_LENGTH,
                          Cf.SLOT_NUM_Y * Cf.SLOT_WIDTH, facecolor='k'))
        plt.text(block.block_location[0] + 5, block.block_location[1] + 5, block_id, fontsize=7, color='w')
        plt.text(block.block_location[0] + 25, block.block_location[1] + 5, str(block.block_location),
                 fontsize=5, color='r')
    exit_loc = Cf.QUAY_EXIT
    plt.plot(exit_loc[0], exit_loc[1], 'o', color='darkgrey', markersize=5)
    plt.text(exit_loc[0], exit_loc[1], 'exit', fontsize=6)
    # plt.savefig(Cf.LAYOUT_PATH)
    plt.show()


if __name__ == '__main__':
    instance = read_input('train', 0, 'Z2')
    plot_layout(instance.init_env)
    #     input_data = input_process.read_json_from_file(Cf.OUTPUT_PATH)
    #     stations = input_process.read_lock_stations_info(input_data.get('lock_stations'))
    #     a = find_latest_machine(stations)
    #     b = find_earliest_machine(stations)
    #     # ma, mb = random_choice_adjacent_two_mission_one_machine(a)
    #     # del_station_afterwards()
