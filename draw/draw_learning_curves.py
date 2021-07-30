#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from drawer import draw
from retriever import retrieve
from processor import process

if __name__ == "__main__":
    database = "exp_official_try_1"
    colls = [
        "asc1", "asc2", "asc3",
        "des1", "des2", "des3",
        "asc_rand1", "asc_rand2", "asc_rand3",
        "rand_des1", "rand_des2", "rand_des3"
    ]
    # episode after which we start penalty adding
    cut = 30
    alpha = 0.8
    fields = ["reward"]  # ["reward", "deviation", "angular_m"]  #
    corrs = {
        "Asc-inc-cog": (0, 3),   # "Ascent": (0, 3),
        "Des-inc-ang": (3, 3),  # "Descent": (3, 3),
        "Asc-uni-cog": (6, 3),  # "Random ascent": (6, 3),
        "Des-uni-ang": (9, 3)  # "Random descent": (9, 3)
    }  # "Ascent": (0, 3), "Descent": (3, 3), "Random ascent": (6, 3)
    raw_data = retrieve(database, colls, fields)
    print("Raw data retrieved")
    processed_data = process(raw_data, cut, alpha)
    print("Data processed")
    draw(processed_data, corrs)
    print("Drawn!")
