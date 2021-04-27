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
        self.penalty_history = []
        self.window = 10
        self.threshold = 0.01  # 0.9
        self.done = False
        self.penalize = False
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

    def update(self, progress, reshaped_penalty, penalty, time_steps):
        print("Update called", progress, penalty)
        self.add_penalty(penalty)
        self.add_time_step(time_steps)
        self.epsilon = self.estimate_epsilon(progress, reshaped_penalty)
        self.epsilon = np.clip(self.epsilon, 0.0, 1.0)
        print("Clipped eps", self.epsilon)
        if self.epsilon > self.threshold and self.level < self.max_level:
            self.level += 1
            self.reward_history = []
            self.send_log(f"{self.seq} level changed {self.level}")
        if self.epsilon > self.threshold and self.level == self.max_level:
            self.send_log(f"{self.seq} penalty added")
            self.penalize = True
            self.reward_history = []
        if self.penalize and self.epsilon > self.threshold and self.level == self.max_level:
            self.send_log(f"{self.seq} learning finished")
            self.done = True
        print(f"Epsilon estimation {self.epsilon}, level {self.level}, pen {self.penalize}, done {self.done}")

    def estimate_epsilon(self, progress, reshaped_penalty):
        print("Appending to history", progress, reshaped_penalty)
        self.reward_history.append(progress-reshaped_penalty)
        if len(self.reward_history) > self.size:
            self.reward_history.pop(0)
        if len(self.reward_history) == 0:
            epsilon = 0.0
        else:
            epsilon = np.mean(self.reward_history[-self.window:])
            self.send_log(f"{self.seq} epsilon {self.epsilon}")
        print("Estimated eps", epsilon)
        return epsilon

    def add_time_step(self, tstep):
        if len(self.time_steps) < self.size:
            self.time_steps.append(tstep)

    def add_penalty(self, penalty):
        if len(self.penalties) < self.size and penalty != 0.:
            self.penalties.append(penalty)

    def estimate_coefficients(self):
        if len(self.penalties) < 70:
            self.penalty = 1.0
            self.duration = 70.
            return None
        self.penalty = np.mean(self.penalties) + 2 * np.std(self.penalties)
        self.duration = np.mean(self.time_steps) + 2 * np.std(self.time_steps)
        self.estimated = True

    def reshape_penalty(self, penalty):
        if not self.penalize:
            return 0.
        elif not self.estimated:
            self.estimate_coefficients()
        penalty = penalty / self.penalty
        penalty = np.clip(penalty / self.duration, 0.0, 1.0 / self.duration)
        return penalty
