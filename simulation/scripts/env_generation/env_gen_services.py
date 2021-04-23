#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from representation import Env
import tf
import time
from simulation.srv import EnvGen, EnvGenResponse
from simulation.srv import GoalInfo, GoalInfoResponse
from simulation.srv import StairInfo, StairInfoResponse
import render
import numpy as np

class EnvGenerator:
    def __init__(self):
        # Wait time to prevent node crashing
        time.sleep(1)
        rospy.init_node('env_gen_services')
        self.env = Env()
        s = rospy.Service('env_gen', EnvGen, self.router)
        s = rospy.Service('stair_info', StairInfo, self.send_stair_info)
        s = rospy.Service('goal_info', GoalInfo, self.send_goal_info)
        self.br = tf.TransformBroadcaster()
        self.generation_mapping = {
            "ground_obstacles": self.generate_ground_obstacles,
            "stair_floor": self.generate_stair_floor,
            "floor_obstacles": self.generate_floor_obstacles,
            "goal": self.generate_goal,
        }
        self.env_mapping = {
            "ground_obstacles": self.env.ground_obstacles,
            "stair_floor": self.env.stair_floor,
            "floor_obstacles": self.env.floor_obstacles,
            "goal": self.env.goal,
        }
        rospy.wait_for_service('/gazebo/delete_model')
        print('The service /gazebo/delete_model is available.')
        for key in self.env_mapping.keys():
            render.delete_model(key)

        while not rospy.is_shutdown():
            self.broadcast_goal()
            self.broadcast_stair()
            rospy.sleep(0.25)

    def send_goal_info(self, _):
        return GoalInfoResponse(
            x=self.env.goal.x,
            y=self.env.goal.y,
            z=self.env.goal.z,
            task=self.env.goal.task,
            rand=self.env.goal.rand
        )

    def send_stair_info(self, _):
        return StairInfoResponse(
            length=self.env.stair_floor.step_length,
            height=self.env.stair_floor.step_height,
            number=self.env.stair_floor.step_n,
            exist=self.env.stair_floor.exist,
        )

    def router(self, req):
        action = req.action
        model_name = req.model
        if model_name not in self.generation_mapping.keys():
            return EnvGenResponse(result=False, err="No such model")
        if action == "generate":
            self.generation_mapping[model_name](req.props)
        elif action == "delete":
            if self.env_mapping[model_name].exist:
                self.env_mapping[model_name].exist = False
                render.delete_model(model_name)
        else:
            return EnvGenResponse(result=False, err="No such action")
        return EnvGenResponse(result=True, err="")

    def generate_ground_obstacles(self, props=None):
        if self.env.ground_obstacles.exist:
            render.delete_model(self.env.ground_obstacles.name)
        self.env.ground_obstacles.generate()
        render.apply(self.env.ground_obstacles)

    def generate_goal(self, props):
        if self.env.goal.exist:
            render.delete_model(self.env.goal.name)
        task, rand = props.split("_")
        rand = bool(int(rand))
        self.env.goal.generate(task, rand)
        render.apply(self.env.goal)

    def broadcast_goal(self):
        self.br.sendTransform(
            [self.env.goal.x, self.env.goal.y, self.env.goal.z],
            [1., 0., 0., 0.],
            rospy.Time.now(),
            "goal",
            "map"
        )

    def broadcast_stair(self):
        """
        length=self.env.stair_floor.step_length,
        height=self.env.stair_floor.step_height,
        number=self.env.stair_floor.step_n,
        exist=self.env.stair_floor.exist,
        :return:
        """
        if not self.env.stair_floor.exist:
            return None
        # unturned frame
        rot = tf.transformations.quaternion_from_euler(0, 0, 0)
        # inclined frame
        a = np.arctan(self.env.stair_floor.step_height / self.env.stair_floor.step_length)
        inclined = tf.transformations.quaternion_from_euler(0, -a, 0)
        # calculate the points
        points = [
            # central frame
            [0, 0, self.env.stair_floor.step_height],
            # central inclined
            [0, 0, 0],
            # up central
            [
                self.env.stair_floor.step_length * (self.env.stair_floor.step_n - 1),
                0,
                self.env.stair_floor.step_height * (self.env.stair_floor.step_n - 1)
            ],
            # up central, inclined
            [0, 0, 0],
            # left
            [0, 1, 0],
            # right
            [0, -1, 0],
            # up left
            [0, 1, 0],
            # up right
            [0, -1, 0],
        ]
        rots = [
            rot, inclined, rot, inclined, rot, rot, rot, rot
        ]
        child = [
            "p_cent", "p_cent_inc", "p_cent_up", "p_cent_up_inc", "p_left", "p_right", "p_left_up", "p_right_up"
        ]
        parent = [
            "map", "p_cent", "p_cent", "p_cent_up", "p_cent", "p_cent", "p_cent_up", "p_cent_up"
        ]
        for i, p in enumerate(points):
            self.br.sendTransform(
                p,
                rots[i],
                rospy.Time.now(),
                child[i],
                parent[i]
            )

    def generate_floor_obstacles(self, props=None):
        if self.env.floor_obstacles.exist:
            render.delete_model(self.env.floor_obstacles.name)
        if not self.env.stair_floor.exist:
            self.generate_stair_floor()
        self.env.floor_obstacles.generate()
        render.apply(self.env.floor_obstacles)

    def update_shift(self):
        self.env.floor_obstacles.shift_x = self.env.stair_floor.shift_x
        self.env.floor_obstacles.shift_z = self.env.stair_floor.shift_z
        self.env.goal.shift_x = self.env.stair_floor.shift_x
        self.env.goal.shift_z = self.env.stair_floor.shift_z

    def generate_stair_floor(self, props=None):
        if self.env.floor_obstacles.exist:
            render.delete_model(self.env.floor_obstacles.name)
        if self.env.stair_floor.exist:
            render.delete_model(self.env.stair_floor.name)
        self.env.stair_floor.generate(props)
        self.update_shift()
        render.apply(self.env.stair_floor)


if __name__ == "__main__":
    EnvGenerator()
