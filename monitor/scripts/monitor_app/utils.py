#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class DictToStruct:
    def __init__(self, **entries):
        """

        :rtype: object
        """
        self.__dict__.update(entries)


class RolloutState:
    """
    Rollout state description.
    """
    def __init__(self):
        self.exp_series = ""
        self.experiment = ""
        self.seq = 0
        self.sensors = ""
        self.arm = True
        self.angular = True
        self.time_step_limit = 100
        self.use_penalty_deviation = False
        self.use_penalty_angular = False
        self.progress = 0.
        self.episode_reward = 0.
        self.step_reward = 0.
        self.progress = 0.
        self.step_deviation = []
        self.step_angular = []
        self.episode_deviation = []
        self.episode_angular = []
        self.done = False
        self.tipping_over_reward = 0
        self.started = False
        self.closest_distance = 1000.0
        self.maximum_distance = 0.0
        self.published = False
        self.log = ""
        self.accidents = ""
        self.tipped = False
        self.time_step = 0

    def reset(self):
        self.__init__()

    def set_fields(self, fields):
        """
        NewRollout.srv request.
        The method stores data from request into the rollout state.
        :param fields: NewRolloutRequest
        :return:
        """
        for field_name in dir(fields):
            if not field_name.startswith("__"):
                setattr(self, field_name, getattr(fields, field_name))


class Sources:
    """
    Enum that helps to indicate what callback function called update_rollout_state.
    """
    robot_state = 1
    deviation = 2
    angular = 3
    odometry = 4


def get_distance(p, q):
    """
    Get distance between two points.
    :param p: an object with x, y, z attributes
    :param q: an object with x, y, z attributes
    :return:
    """
    # Distance to goal where experiment ends.
    shift = 0.3
    return ((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2) ** 0.5 - shift
