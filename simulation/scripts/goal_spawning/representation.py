#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import math
import copy
from abc import ABC, abstractmethod

class Sphere:
    def __init__(self, x, y, z, r, t=.5):
        self.x = x
        self.y = y
        self.z = z
        self.transparency = t
        self.radius = r

class Goal:
    def __init__(self, x, y, z):
        self.inner = Sphere(x, y, z, 0.1, 0.)
        self.outter = Sphere(x, y, z, 0.3, 0.5)
