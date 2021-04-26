#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
The guidance class that ensures progressive learning.
"""

import numpy as np

class Guidance:
    def __init__(self):
        # 0.0 - 1.0
        self.epsilon = 0.
        # 0 - 2
        self.level = 0
        self.max_level = 2
        self.reward_history = []
        self.window = 10
        self.threshold = 0.9
        self.done = False

    def update(self, reward):
        self.reward_history.append(reward)
        if len(self.reward_history) == 0:
            self.epsilon = 0.0
        else:
            self.epsilon = np.mean(self.reward_history[-self.window:])
        if self.epsilon > self.threshold and self.level < self.max_level:
            self.level += 1
        if self.epsilon > self.threshold and self.level == self.max_level:
            self.done = True

class Penalty:
    def __init__(self, ):
        self.k = 1.0
        self.estimated = False
        self.time_steps = []
        self.penalties = []
        self.size = 10

    def get_coefficient(self):
        return self.k

    def add_time_step(self, tstep):
        if len(self.time_steps) < self.size:
            self.time_steps.append(tstep)

    def add_penalty(self, penalty):
        if len(self.penalties) < self.size:
            self.penalties.append(penalty)
        if not self.estimated and len(self.penalties) >= self.size:
            self.coef = 1.0 / np.max(self.time_steps) / 2

    def reshape_penalty(self, penalty):
        penalty = np.clip(penalty, -self.coef, 0.0)
        return penalty

