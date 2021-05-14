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
        self.max_level = 2
        self.need_to_penalize = need_to_penalize
        self.reward_history = []
        self.penalty_history = []
        self.done = False
        self.penalize = False
        self.complexity_type = None
        # penalty part
        self.estimated = False
        self.time_steps = []
        self.penalties = []
        self.duration = 0.
        self.penalty = 0.
        # Hyper parameters
        # define the size of array on what we calculate penalty terms
        self.size_penalty = 20  # 100
        # reward history_size, in fact, it is not a hyper parameter, it is just RAM constraint
        self.size_reward_history = 20  # 20
        # Moving average window for competence estimation
        self.window_epsilon = 10
        # threshold that defines when to increment complexity or stop learning
        self.epsilon_threshold = 0.82  # 0.9
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

    def update(self, progress, penalty, time_steps):
        # print("Update called", progress, penalty)
        self.add_penalty(penalty)
        self.add_time_step(time_steps)
        self.epsilon = self.estimate_epsilon(progress, penalty)
        self.epsilon = np.clip(self.epsilon, 0.0, 1.0)

        if self.seq % 10:
            self.send_log(f"current epsilon {self.epsilon}, complexity {self.level}")
        # incremental complexity
        if self.complexity_type == "inc":
            if self.epsilon > self.epsilon_threshold \
                    and self.level < self.max_level \
                    and len(self.reward_history) >= self.size_reward_history:
                self.level += 1
                self.reward_history = []
                self.epsilon = 0
                self.send_log(f"level changed {self.level}")
            elif self.epsilon > self.epsilon_threshold \
                    and self.level == self.max_level \
                    and len(self.reward_history) >= self.size_reward_history:
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
                    and len(self.reward_history) >= self.size_reward_history:
                self.send_log(f"exp done with penalty")
                self.done = True
        # full complexity
        elif self.complexity_type == "full":
            # simple condition to exit
            if self.epsilon > self.epsilon_threshold and len(self.reward_history) >= self.size_reward_history:
                self.done = True

        # print(f"Epsilon estimation {self.epsilon}, level {self.level}, pen {self.penalize}, done {self.done}")

    def set_need_to_penalize(self, value):
        self.need_to_penalize = value

    def get_need_to_penalize(self):
        return self.need_to_penalize

    def estimate_epsilon(self, progress, penalty):
        # print("Appending to history", progress, reshaped_penalty)
        penalty = self.reshape_penalty(penalty)
        if not self.penalize:
            penalty = 0.

        self.reward_history.append(progress-penalty)
        if len(self.reward_history) > self.size_reward_history:
            self.reward_history.pop(0)

        if len(self.reward_history) == 0:
            epsilon = 0.0
        else:
            epsilon = np.mean(self.reward_history[-self.window_epsilon:])
            self.send_log(f"{datetime.datetime.now().ctime()}: {self.seq}: epsilon {self.epsilon}")
        print("Estimated eps", epsilon)
        return epsilon

    def add_time_step(self, tstep):
        if len(self.time_steps) < self.size_penalty:
            self.time_steps.append(tstep)

    def add_penalty(self, penalty):
        if len(self.penalties) < self.size_penalty and penalty != 0.:
            self.penalties.append(penalty)

    def estimate_coefficients(self):
        if len(self.penalties) < self.size_penalty:
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
