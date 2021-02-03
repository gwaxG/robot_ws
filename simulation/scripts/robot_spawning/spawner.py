#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import math
import copy
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from simulation.srv import StairInfo
from gazebo_msgs.msg import ModelState

class Spawner:
    ground_width = 10.
    stair_width = 2.

    def __init__(self):
        self.stair_info = rospy.ServiceProxy('stair_info', StairInfo)
        self.gazebo_model_state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.roll = 0.
        self.pitch = 0.
        self.yaw = 0.
        self.step_n = 0.
        self.step_length = 0.
        self.step_height = 0.

    def init(self):
        """
        Reset robot pose to zero.
        Retrieve information about thestaircase.
        :return:
        """
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.roll = 0.
        self.pitch = 0.
        self.yaw = 0.

        res = self.stair_info.call()
        if res.exist:
            self.step_length = res.length
            self.step_height = res.height
            self.step_n = res.number
        return self

    def set_place(self, place):
        """
        We set position of the robot either on the ground or on the floor.
        :param place: where to spawn
        :return:
        """
        if place == "ground":
            self.x = -1.0
            self.z = 1.0
        elif place == "floor":
            self.x = 1.0 + self.step_length * self.step_n
            self.z = 1.0 + self.step_height * self.step_n
        return self

    def set_task(self, task):
        """
        Roboot rotation accordingly to the task.
        Basically, it has to be orientated to the traversing object.
        :param task:
        :return:
        """
        if task == "ascent":
            pass
        elif task == "descent":
            self.yaw = math.pi
        elif task == "flat":
            if self.x < 0.:
                self.yaw = math.pi
            else:
                pass
        return self

    def set_randomness(self, rand, task):
        """
        Define if random yaw and position along y axis.
        :param rand:
        :return:
        """
        if task == "ascent" or task == "descent":
            delta = Spawner.stair_width - .5
        elif task == "flat":
            delta = Spawner.ground_width - 2.

        if int(rand):
            self.y = delta * (0.5-random.random())
            self.yaw += (2 * random.random()-1.) * math.pi / 2
        return self

    def spawn(self):
        msg = ModelState()
        msg.model_name = "jaguar"
        msg.twist = Twist()
        msg.pose = Pose()
        msg.reference_frame = "world"

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.gazebo_model_state_pub.publish(msg)
