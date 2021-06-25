#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from drawer import draw
from retriever import retrieve
from processor import process

if __name__ == "__main__":
    database = "exp_of_2"
    colls = ["asc1"]
    # episode after which we start penalty adding
    cut = 30
    alpha = 0.99
    fields = ["deviation", "debug", "reward"]
    corrs = {"Ascent": (0, 1)}
    raw_data = retrieve(database, colls, fields)
    print("Raw data retrieved")
    processed_data = process(raw_data, cut, alpha)
    print("Data processed")
    draw(processed_data, corrs)
    print("Drawn!")
