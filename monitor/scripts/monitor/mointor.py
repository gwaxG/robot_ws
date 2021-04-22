#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

"""

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from simulation.srv import StairInfo, StairInfoRequest
from control.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

import tf
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np


class Monitor:
    def __init__(self):
        rospy.init_node('monitor')


if __name__ == '__main__':
    Monitor()
