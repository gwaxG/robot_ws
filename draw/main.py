#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from drawer import draw
from retriever import retrieve
from processor import process

if __name__ == "__main__":
    database = "exp_official_try_1"
    colls = ["asc1", "asc2", "asc3", "des1", "des2", "des3"]  #
    # episode after which we start penalty adding
    cut = 30
    alpha = 0.8
    fields = ["deviation"]  #
    corrs = {"Ascent": (0, 3)}  # "Ascent": (0, 3), "Descent": (3, 3)
    raw_data = retrieve(database, colls, fields)
    print("Raw data retrieved")
    processed_data = process(raw_data, cut, alpha)
    print("Data processed")
    draw(processed_data, corrs)
    print("Drawn!")
