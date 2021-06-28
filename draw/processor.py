#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import scipy.interpolate as interp


def apply_ema(y, alpha=0.99):
    """
    EMA for an 1D array
    :param y:
    :return:
    """
    st = y[0]
    y_ = [y[0]]
    for i in range(1, len(y)):
        st = alpha * st + (1 - alpha) * y[i]
        y_.append(st)
    return y_


def ema(data, alpha):
    """
    Moving average smoothing.
    :param data:
    :return:
    """
    smoothed = []
    for arr in data:
        sm_arr = apply_ema(arr, alpha)
        smoothed.append(sm_arr)
    return smoothed


def align(data):
    """
    :param data: list of arrays to align
    :return: aligned arrays
    """
    aligned = []
    lengths = [len(arr) for arr in data]
    min_length = min(lengths)
    ref = lengths.index(min_length)
    arr_ref = data[ref]
    # We compress all arrays to the reference one.
    for i, arr in enumerate(data):
        if i != ref:
            arr_interp = interp.interp1d(np.arange(arr.size), arr)
            arr_compress = arr_interp(np.linspace(0, arr.size - 1, arr_ref.size))
            aligned.append(arr_compress)
    aligned.append(arr_ref)
    return aligned


def align2(data):
    n = 101
    aligned = []
    for i, arr_ in enumerate(data):
        # Ad-hoc solution of 2 last broken episodes.
        arr = arr_[:-2]
        arr_interp = interp.interp1d(np.arange(arr.size), arr)
        arr_compress = arr_interp(np.linspace(0, arr.size - 1, n))
        aligned.append(arr_compress)
    return aligned


def cut_data(data, cut):
    for k, v in data.items():
        values = []
        for val in v:
            values.append(val[cut:])
        data[k] = values
    return data


def filter_deviation(data):
    dev_arrays = []
    for arrays in data["deviation"]:
        values = []
        for elem in arrays:
            if elem > 0.05:
                values.append(elem)
            else:
                if len(values) == 0:
                    values.append(0.32)
                else:
                    values.append(np.mean(values[-10:]))
        dev_arrays.append(np.array(values))
    data["deviation"] = dev_arrays
    return data


def process(data, cut, alpha, way="ema"):
    """
    :param data: dictionary where keys are experiment metrics and values are episode metric values
    :return:
    """
    aligned = {}
    smoothed = {}
    data = cut_data(data, cut)
    if "deviation" in data.keys():
        data = filter_deviation(data)
    # align
    for k, v in data.items():
        aligned[k] = align2(v)
    # smooth
    for k, v in aligned.items():
        smoothed[k] = getattr(sys.modules[__name__], way)(v, alpha)
    return smoothed
