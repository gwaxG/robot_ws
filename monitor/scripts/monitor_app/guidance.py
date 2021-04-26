#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
The guidance class that ensures progressive learning.
"""

import numpy as np


class Guidance:
    def __init__(self):
        """
        Tasks:
            increase complexity, increase size, increase complexity, increase size, increase complexity, increase size,
            increase complexity, add penalty
        """
        # Guidance part
        # 0.0 - 1.0
        self.epsilon = 0.
        # 0 - 2
        self.level = 0
        self.max_level = 2
        self.reward_history = []
        self.window = 10
        self.threshold = 0.01  # 0.9
        self.done = False
        self.add_penalty = False
        # penalty part
        self.estimated = False
        self.time_steps = []
        self.penalties = []
        self.size = 1  # 100
        self.duration = 0.
        self.penalty = 0.
        # Sync data
        self.log_update = False
        self.log_string = ""
        self.seq = 0

    def set_seq(self, seq):
        self.seq = seq

    def reset_sync_log(self):
        self.log_update = False
        self.log_string = ""

    def send_log(self, msg):
        self.log_string = msg
        self.log_update = True

    def update(self, reward):
        self.reward_history.append(reward)
        if len(self.reward_history) == 0:
            self.epsilon = 0.0
        else:
            self.epsilon = np.mean(self.reward_history[-self.window:])
            self.send_log(f"{self.seq} epsilon {self.epsilon}")
        if self.epsilon > self.threshold and self.level < self.max_level:
            self.level += 1
            self.send_log(f"{self.seq} level changed {self.level}")
        if self.epsilon > self.threshold and self.level == self.max_level:
            self.send_log(f"{self.seq} penalty added")
            self.add_penalty = True
        if self.add_penalty and self.epsilon > self.threshold and self.level == self.max_level:
            self.send_log(f"{self.seq} learning finished")
            self.done = True

    def add_time_step(self, tstep):
        if len(self.time_steps) < self.size:
            self.time_steps.append(tstep)

    def add_penalty(self, penalty):
        if len(self.penalties) < self.size:
            self.penalties.append(penalty)
        if not self.estimated and len(self.penalties) >= self.size:
            self.penalty = np.mean(self.penalties) + 2 * np.std(self.penalties)
            self.duration = np.mean(self.time_steps) + 2 * np.std(self.time_steps)
            self.estimated = True

    def reshape_penalty(self, penalty):
        if not self.add_penalty:
            return 0.
        penalty = penalty / self.penalty
        penalty = np.clip(penalty / self.duration, 0.0, 1.0 / self.duration)
        return penalty
