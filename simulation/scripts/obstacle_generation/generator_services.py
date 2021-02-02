#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_srvs.srv import Trigger, TriggerResponse
import rospy
from representation import Env
import render


class EnvGenerator:
    def __init__(self):
        rospy.init_node('env_generator')
        self.env = Env()
        s = rospy.Service('generate_ground_obstacles', Trigger, self.generate_ground_obstacles)
        s = rospy.Service('generate_floor_obstacles', Trigger, self.generate_floor_obstacles)
        s = rospy.Service('generate_stair_floor', Trigger, self.generate_stair_floor)
        rospy.spin()

    def generate_ground_obstacles(self, req):
        self.env.ground_obstacles.generate()
        render.apply(self.env.ground_obstacles)
        return TriggerResponse(True, "")

    def generate_floor_obstacles(self, req):
        self.env.floor_obstacles.generate()
        render.apply(self.env.floor_obstacles)
        return TriggerResponse(True, "")

    def generate_stair_floor(self, req):
        self.env.stair_floor.generate()
        self.env.floor_obstacles.shift_x = self.env.stair_floor.shift_x
        self.env.floor_obstacles.shift_z = self.env.stair_floor.shift_z
        render.apply(self.env.stair_floor)
        return TriggerResponse(True, "")


if __name__ == "__main__":
    EnvGenerator()