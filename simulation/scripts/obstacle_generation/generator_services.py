#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from representation import Env
import render
from simulation.srv import EnvGen, EnvGenResponse
from simulation.srv import StairInfo, StairInfoResponse
import render

class EnvGenerator:
    def __init__(self):
        rospy.init_node('env_generator')
        self.env = Env()
        s = rospy.Service('env_gen', EnvGen, self.router)
        s = rospy.Service('stair_info', StairInfo, self.send_stair_info)
        self.generation_mapping = {
            "ground_obstacles": self.generate_ground_obstacles,
            "stair_floor": self.generate_stair_floor,
            "floor_obstacles": self.generate_floor_obstacles,
        }
        self.env_mapping = {
            "ground_obstacles": self.env.ground_obstacles,
            "stair_floor": self.env.stair_floor,
            "floor_obstacles": self.env.floor_obstacles,
        }
        for key in self.env_mapping.keys():
            render.delete_model(key)
        rospy.spin()

    def send_stair_info(self, _):
        return StairInfoResponse(
            length = self.env.stair_floor.step_length,
            height=self.env.stair_floor.step_height,
            number=self.env.stair_floor.step_n,
            exist=self.env.stair_floor.exist
        )

    def router(self, req):
        action, model_name = req.action.split("/")[1:]
        if model_name not in self.generation_mapping.keys():
            return EnvGenResponse(result=False, err="No such model")
        if action == "generate":
            self.generation_mapping[model_name]()
        elif action == "delete":
            if self.env_mapping[model_name].exist:
                self.env_mapping[model_name].exist = False
                render.delete_model(model_name)
        else:
            return EnvGenResponse(result=False, err="No such action")
        return EnvGenResponse(result=True, err="")

    def generate_ground_obstacles(self):
        if self.env.ground_obstacles.exist:
            render.delete_model(self.env.ground_obstacles.name)
        self.env.ground_obstacles.generate()
        render.apply(self.env.ground_obstacles)

    def generate_floor_obstacles(self):
        if self.env.floor_obstacles.exist:
            render.delete_model(self.env.floor_obstacles.name)
        if not self.env.stair_floor.exist:
            self.generate_stair_floor()
        self.env.floor_obstacles.generate()
        render.apply(self.env.floor_obstacles)

    def generate_stair_floor(self):
        if self.env.floor_obstacles.exist:
            render.delete_model(self.env.floor_obstacles.name)
        if self.env.stair_floor.exist:
            render.delete_model(self.env.stair_floor.name)
        self.env.stair_floor.generate()
        self.env.floor_obstacles.shift_x = self.env.stair_floor.shift_x
        self.env.floor_obstacles.shift_z = self.env.stair_floor.shift_z
        render.apply(self.env.stair_floor)


if __name__ == "__main__":
    EnvGenerator()