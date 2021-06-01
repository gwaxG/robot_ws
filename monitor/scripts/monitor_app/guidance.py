#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
The guidance class that ensures progressive learning.
"""

import numpy as np
import datetime
import utils


class Guidance:
    def __init__(self, need_to_penalize=False):
        """
        Tasks:
            increase complexity, increase size, increase complexity, increase size, increase complexity, increase size,
            increase complexity, add penalty
        """
        # Guidance part
        # 0.0 - 1.0
        self.epsilon = 0.
        # 0 - 2
        self.levels = utils.DictToStruct(**{"min": 0, "max": 2})
        self.level = None  # we initialize compl. level in the set_complexity_type method
        self.max_level = 1
        self.need_to_penalize = need_to_penalize
        self.reward_history = []
        self.done = False
        self.penalize = False  # False
        self.complexity_type = None
        # penalty part
        self.reshaping_coefficient = 0.
        self.used_penalty = []
        # Hyper parameters
        # Moving average window for competence estimation
        self.window_epsilon = 20  # 10
        self.start_size = 20
        # threshold that defines when to increment complexity or stop learning
        self.epsilon_threshold = 0.82  # 0.82
        # Sync data
        self.log_update = False
        self.log_string = ""
        self.seq = 0

    def set_complexity_type(self, t):
        self.complexity_type = t
        if self.complexity_type == "inc":
            self.level = self.levels.min
        else:
            self.level = self.levels.max

    def set_seq(self, seq):
        self.seq = seq

    def reset_sync_log(self):
        self.log_update = False
        self.log_string = ""

    def send_log(self, msg):
        self.log_string = msg
        self.log_update = True

    def safety_angular(self, relative_value):
        self.reshaping_coefficient = (1. - relative_value)

    def safety_deviation(self, relative_value):
        self.reshaping_coefficient = (1. - relative_value)

    def update(self, progress, time_steps):
        self.used_penalty = []
        self.add_to_history(progress, time_steps)
        self.epsilon = self.estimate_epsilon()
        print("aftermath", self.epsilon)

        # TODO to delete
        if self.seq % 10 == 0 and False:
            self.send_log(f"current epsilon {self.epsilon}, complexity {self.level}")
        # incremental complexity
        if self.complexity_type == "inc":
            if self.epsilon > self.epsilon_threshold \
                    and self.level < self.max_level \
                    and len(self.reward_history) >= self.start_size:
                self.level += 1
                self.reward_history = []
                self.epsilon = 0
                self.send_log(f"level changed to {self.level}")
            elif self.epsilon > self.epsilon_threshold \
                    and self.level == self.max_level \
                    and len(self.reward_history) >= self.start_size \
                    and not self.penalize:
                # penalization inclusion
                if self.need_to_penalize:
                    self.penalize = True
                    self.send_log(f"penalty added")
                else:
                    self.done = True
                    self.send_log(f"exp done without penalty")
                self.reward_history = []
            elif self.penalize \
                    and self.epsilon > self.epsilon_threshold \
                    and len(self.reward_history) >= self.start_size:
                self.send_log(f"exp done with penalty")
                self.done = True
        # full complexity
        elif self.complexity_type == "full":
            # simple condition to exit
            if self.epsilon > self.epsilon_threshold and len(self.reward_history) >= self.start_size:
                if self.need_to_penalize and not self.penalize:
                    self.penalize = True
                    self.reward_history = []
                else:
                    self.done = True
        print(
            f"Epsilon estimation {self.epsilon}, length {len(self.reward_history)}, pen {self.penalize}, done {self.done}")

    def set_need_to_penalize(self, value):
        self.need_to_penalize = value

    def get_need_to_penalize(self):
        return self.need_to_penalize

    def add_to_history(self, progress, time_steps):
        self.reward_history.append(progress)

    def estimate_epsilon(self):
        if len(self.reward_history) == 0:
            epsilon = 0.0
        else:
            epsilon = np.mean(self.reward_history[-self.window_epsilon:])
        epsilon = np.clip(epsilon, 0.0, 1.0)
        return epsilon

    def get_epsilon(self):
        return self.epsilon

    def reshape_reward(self, reward):
        if not self.penalize:
            return reward
        print("reshaping", reward, self.reshaping_coefficient)
        self.used_penalty.append(self.reshaping_coefficient)
        return self.reshaping_coefficient * reward
