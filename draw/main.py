#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from drawer import draw
from retriever import retrieve
from processor import process

if __name__ == "__main__":
    database = "exp_17_06"
    colls = ["asc", "asc2"]
    fields = ["deviation", "debug", "reward"]
    corrs = {"Ascent": (0, 2)}
    raw_data = retrieve(database, colls, fields)
    print("Raw data retrieved")
    processed_data = process(raw_data)
    print("Data processed")
    draw(processed_data, corrs)
    print("Drawn!")
