#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import numpy as np
import scipy.interpolate as interp


def apply_ema(y, alpha=0.95):
    """
    EMA for an 1D array
    :param y:
    :return:
    """
    st = 0
    y_ = []
    for i, el in enumerate(y):
        st = alpha * st + (1 - alpha) * el
        y_.append(st)
    return y_


def ema(data):
    """
    Moving average smoothing.
    :param data:
    :return:
    """
    smoothed = []
    for arr in data:
        sm_arr = apply_ema(arr)
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


def process(data, way="ema"):
    """
    :param data: dictionary where keys are experiment metrics and values are episode metric values
    :return:
    """
    aligned = {}
    smoothed = {}
    # align
    for k, v in data.items():
        aligned[k] = align(v)
    # smooth
    for k, v in aligned.items():
        smoothed[k] = getattr(sys.modules[__name__], way)(v)
    return smoothed
