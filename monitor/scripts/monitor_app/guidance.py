#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
The guidance class that ensures progressive learning.
"""

import numpy as np
import datetime
import utils


class Guidance:
    def __init__(self, penalty_type="free"):
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
        self.progress = 0.
        self.need_to_penalize = False
        self.reward_history = []
        self.progress_history = []
        self.done = False
        self.complexity_type = None
        # penalty part
        self.used_penalty = [[]]
        self.normalization = 1.
        self.normalized = False
        # Hyper parameters
        # Moving average window for competence estimation
        self.window_epsilon = 20  # 10
        self.start_size = 30
        # threshold that defines when to increment complexity or stop learning
        # 0.99 is good when there is no penalties, otherwise 0.9 is better
        epsilon_thresholds = {
            "free": 0.9,
            "deviation": 0.85,  # 0.75
            "angular": 0.6,  # 0.5
        }
        self.penalty_type = penalty_type
        self.epsilon_threshold = epsilon_thresholds[penalty_type]
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

    def safety_push(self, relative_value):
        self.queue.push(relative_value)

    def update(self, episode_reward, progress):
        """
        Exit by reward estimation.
        Increment complexity by progress estimation.
        :param episode_reward:
        :param progress:
        :return:
        """
        msg = ""
        if len(self.used_penalty) < self.start_size and self.need_to_penalize:
            self.used_penalty.append([])
        elif not self.normalized and self.need_to_penalize:
            self.normalization = 1.0 / np.mean([np.sum(arr) for arr in self.used_penalty])
            msg += f"Normalized with K={self.normalization}!"
            self.normalized = True
        self.reward_history.append(episode_reward)
        self.progress_history.append(progress)
        self.epsilon = self.estimate_epsilon()
        msg += f"Current epsilon {self.epsilon}"
        self.progress = self.estimate_progress()
        done = False
        # incremental complexity
        # simple condition to exit
        if self.epsilon > self.epsilon_threshold and len(self.reward_history) >= self.start_size:
            if len(self.reward_history) >= 2 * self.start_size:
                msg += "Done with penalty\n"
                done = True
            else:
                msg += "Threshold touched, time step number is not\n"
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
        if epsilon < 0.:
            self.send_log(f"negative epsilon {epsilon}")
        epsilon = np.clip(epsilon, 0.0, 1.0)
        return epsilon

    def estimate_progress(self):
        if len(self.progress_history) == 0:
            progress = 0.0
        else:
            progress = np.mean(self.progress_history[-self.window_epsilon:])
        progress = np.clip(progress, 0.0, 1.0)
        return progress

    def get_epsilon(self):
        return self.epsilon

    def get_progress(self):
        return self.progress

    def reshape_reward(self, reward):
        # Reward is positive and reflects the travelled distance.
        if not self.normalized and self.need_to_penalize:
            queue_value = self.queue.get_mean_value()
            self.used_penalty[-1].append(queue_value)
        if not self.need_to_penalize or (self.need_to_penalize and not self.normalized):
            return reward
        else:
            queue_value = self.queue.get_mean_value()
            return reward - self.normalization * queue_value
