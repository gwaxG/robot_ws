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
        # Frequency of IMU is 20 Hz, the time steps lasts 0.25 second.
        # Thus, last 5 angular velocity safety updates keep information about last time step.
        self.queue = utils.PassageQueue(size=5)
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
        # 0.99 is good when there is no penalties, otherwise 0.9 is better
        self.epsilon_threshold = 0.9
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
        self.queue.push(relative_value)
        self.reshaping_coefficient = (1. - self.queue.get_mean_value())

    def safety_deviation(self, relative_value):
        self.queue.push(relative_value)
        self.reshaping_coefficient = self.queue.get_mean_value()

    def update(self, episode_reward, time_steps):
        self.used_penalty = []
        self.reward_history.append(episode_reward)
        self.epsilon = self.estimate_epsilon()
        msg = f""
        done = False
        # incremental complexity
        if self.complexity_type == "inc":
            msg += f"current epsilon {np.round(self.epsilon, 2)}/{self.epsilon_threshold}, complexity {self.level}\n"
            if self.epsilon > self.epsilon_threshold \
                    and self.level < self.max_level \
                    and len(self.reward_history) >= self.start_size\
                    and not self.penalize:
                self.level += 1
                self.reward_history = []
                self.epsilon = 0
                msg += f"level change to {self.level}\n"
            elif self.epsilon > self.epsilon_threshold \
                    and self.level >= self.max_level \
                    and len(self.reward_history) >= self.start_size \
                    and not self.penalize:
                # penalization inclusion
                msg += f"level change to reached threshold at maximum level, checking for penalization\n"
                if self.need_to_penalize:
                    self.penalize = True
                    msg += f"penalty added\n"
                else:
                    done = True
                    msg += f"exp done without penalty\n"
                self.reward_history = []
            elif self.penalize \
                    and self.epsilon > self.epsilon_threshold \
                    and len(self.reward_history) >= self.start_size:
                msg += f"exp done with penalty\n"
                done = True
        # full complexity
        elif self.complexity_type == "full":
            # simple condition to exit
            msg += f"check for end\n"
            if self.epsilon > self.epsilon_threshold and len(self.reward_history) >= self.start_size:
                msg += f"Threshold touched\n"
                if self.need_to_penalize:
                    done = True
                    msg += f"Penalties added\n"
                    self.penalize = True
                    self.reward_history = []
                else:
                    msg += f"Done without penalty\n"
                    done = True
                if self.penalize and self.need_to_penalize:
                    msg += f"Done with penalty\n"
                    done = True
        self.send_log(msg)
        self.done = done

    def set_need_to_penalize(self, value):
        self.need_to_penalize = value

    def get_need_to_penalize(self):
        return self.need_to_penalize

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
        # Reward is positive and reflects the travelled distance.
        # kappa~2 means that if safety is more than 0.5 then the robot receives the negative reward which goes down to -1.
        kappa = 2
        self.used_penalty.append(self.reshaping_coefficient)
        if not self.need_to_penalize:
            return reward
        else:
            return reward * (1 - kappa * self.reshaping_coefficient)
