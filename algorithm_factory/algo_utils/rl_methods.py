# -*- coding: utf-8 -*-
from typing import List

import numpy as np
import torch
from prettytable import PrettyTable
from torch import Tensor
from torch import nn
from torch.nn.utils.rnn import pack_padded_sequence


def del_tensor_ele(arr, index):
    if index == 0:
        _, a2 = torch.split(arr, (1, len(arr[0]) - 1), 1)
        return a2
    else:
        a1, _, a2 = torch.split(arr, (index - 1, 1, len(arr[0]) - index), 1)
        return torch.cat((a1, a2), dim=1)


def soft_update(tgt: nn.Module, src: nn.Module, tau: float) -> None:
    """
    Update target net
    """
    for tgt_param, src_param in zip(tgt.parameters(), src.parameters()):
        tgt_param.data.copy_(tau * src_param.data + (1 - tau) * tgt_param.data)


def print_result(field_name: List, value: List) -> None:
    res_table = PrettyTable()
    res_table.field_names = field_name
    res_table.add_row(value)
    print('\n', res_table)


def process_multi_sequence(seq_data):
    """

    :param seq_data: [batch size, machine_number, number of words, number of features]
    :return: iter(PackedSequence) each PackedSequence stands for one machine
    """
    seq_data = torch.tensor(seq_data, dtype=torch.float32)
    if seq_data.dim() != 4:
        seq_data = seq_data.unsqueeze(0)
    seq_ls = torch.split(seq_data, split_size_or_sections=1, dim=1)
    pack_ls = []
    for seq in seq_ls:
        seq = seq.squeeze(1)
        pack = process_single_sequence(seq)
        pack_ls.append(pack)
    return pack_ls


def process_single_sequence(seq: Tensor):
    seq_l = get_sequence_length(seq).numpy()
    order_idx = np.argsort(seq_l)[::-1]
    order_seq = seq[order_idx.tolist()]
    order_seq_l = seq_l[order_idx]
    pack = pack_padded_sequence(order_seq, order_seq_l, batch_first=True)
    return pack


def process_single_state(state):
    s_m, s_s, s_c, s_y = state
    s_m_ = torch.tensor(s_m, dtype=torch.float32)
    s_s_ = process_multi_sequence(s_s)
    s_c_ = process_single_sequence(torch.tensor(s_c, dtype=torch.float32).unsqueeze(0))
    s_y_ = process_single_sequence(torch.tensor(s_y, dtype=torch.float32).unsqueeze(0))
    return s_m_, s_s_, s_c_, s_y_


def get_sequence_length(seq: Tensor) -> Tensor:
    """
    Args:
        seq: sequence data in dimension of [batch size, number of words, number of features]
        Returns: pack as input of RNN
    """
    seq_length = torch.sum((seq != 0.0).any(dim=2), dim=1) + 1.0
    return seq_length
