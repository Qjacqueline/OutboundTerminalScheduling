#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
@Project ：Port_Scheduling_New
@File    ：log.py
@Author  ：JacQ
@Date    ：2022/4/21 10:45
"""
import logging
import logging.handlers
import os
import time
from typing import Optional

from conf import configs


def exp_dir(desc: Optional[str] = None) -> str:
    time_str = time.strftime('%m_%d_%H_%M', time.localtime())
    if desc:
        desc = desc
        file_path = f'./runss/{desc}_{time_str}'
    else:
        file_path = f'./runss/{time_str}'
    if not os.path.exists(file_path):
        os.makedirs(file_path)
    print(f'Result is stored in {file_path}')
    return file_path


class Logger:
    def __init__(self, name="root"):
        self._logger = logging.getLogger(name)
        # formatter = logging.Formatter('[%(asctime)s] [%(levelname)s] - %(filename)s(line:%(lineno)d) - %(message)s',
        #                               '%Y-%m-%d %H:%M:%S')
        formatter = logging.Formatter('[%(asctime)s] [%(levelname)s] - %(message)s',
                                      '%Y-%m-%d %H:%M:%S')

        if not self._logger.handlers:
            dir_path = os.path.dirname(__file__)
            log_dir_path = os.path.abspath(f'{dir_path}/../../Port_Scheduling/logs')
            if not os.path.exists(log_dir_path):
                os.makedirs(log_dir_path)
            info_log_path = os.path.abspath(os.path.join(log_dir_path, 'info.log'))
            warn_log_path = os.path.abspath(os.path.join(log_dir_path, 'warn.log'))
            handler_info = logging.handlers.TimedRotatingFileHandler(info_log_path, when='D', interval=1)
            handler_warn = logging.handlers.TimedRotatingFileHandler(warn_log_path, when='D', interval=1)
            stream_handler = logging.StreamHandler()
            stream_handler.setFormatter(formatter)

            handler_info.setFormatter(formatter)
            handler_warn.setFormatter(formatter)
            handler_info.setLevel(configs.LOGGING_LEVEL)
            handler_warn.setLevel(logging.WARN)

            self._logger.addHandler(handler_info)
            self._logger.addHandler(handler_warn)
            self._logger.addHandler(stream_handler)

            self._logger.setLevel(configs.LOGGING_LEVEL)

    def get_logger(self) -> object:
        return self._logger
