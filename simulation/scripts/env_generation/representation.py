#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import math
import copy
from abc import ABC, abstractmethod

class Env:

    def __init__(self):
        self.ground_obstacles = GroundObstacles(wall_x_size=1.0, wall_y_size=2.0, wall_z_size=2.0)
        self.stair_floor = StairFloor()
        self.floor_obstacles = FloorObstacles(
            wall_x_size=1.0, wall_y_size=2.0, wall_z_size=2.0,
            shift_x=2.0, shift_z=2.0
        )
        self.goal = Goal()


class Group(ABC):
    def __init__(self, name):
        self.name = name
        self.walls = []
        self.floor = None
        self.steps = []
        self.spheres = []

    @abstractmethod
    def generate(self):
        pass

class Box:
    def __init__(self, name, x=0., y=0., z=0., roll=0., pitch=0., yaw=0., box_x=0.01, box_y=0.01, box_z=0.0):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.box_x = box_x
        self.box_y = box_y
        self.box_z = box_z

class Sphere:
    def __init__(self, name, x=0., y=0., z=0.,  radius=0.01, transparency=.5):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.transparency = transparency
        self.radius = radius

class Goal(Group):

    def __init__(self):
        super().__init__("goal")
        self.spheres = []
        self.shift_x = 0.
        self.shift_z = 0.
        self.exist = False

        self.x = 1.99
        self.y = 1.99
        self.z = 0.085

        self.rand = False
        self.task = ""

    def generate(self, task, rand):
        """

        :param task: ascent, descent, flat
        :param rand: moves randomly the goal in the square 0.5m*0.5m + random location along Y for the flat task
        :return:
        """
        self.rand = rand
        self.task = task
        self.x = 0.
        self.y = 0.
        self.z = 0.085

        if task == "ascent":
            self.x += self.shift_x + 1.0
            self.z += self.shift_z
        elif task == "descent":
            self.x += -1.0
        elif task == "flat":
            self.x -= 11.0
            if rand:
                self.y = (0.5 - random.random()) * 8.0
        if rand:
            self.x += (0.5 - random.random()) * 1.0
            self.y += (0.5 - random.random()) * 1.0


        spheres = [
            Sphere(
                name="sphere_inner",
                x=self.x,
                y=self.y,
                z=self.z,
                transparency=0.0,
                radius=0.05,
            ),
            Sphere(
                name="sphere_outter",
                x=self.x,
                y=self.y,
                z=self.z,
                transparency=0.75,
                radius=0.3,
            )
        ]
        self.spheres = spheres
        self.exist = True

class FloorObstacles(Group):

    def __init__(self, wall_x_size, wall_y_size, wall_z_size, shift_x, shift_z):
        super().__init__("floor_obstacles")
        self.wall_z_size = wall_z_size
        self.wall_x_size = wall_x_size
        self.wall_y_size = wall_y_size
        self.shift_x = shift_x
        self.shift_z = shift_z
        self.exist = False

    def generate(self):
        walls = []
        for dist_i in range(4):
            dist = 3.0 + dist_i*2 + self.shift_x
            for wall_i in range(2):
                walls.append(
                    Box(
                        name="wall_"+str(dist_i) + "_" + str(wall_i),
                        x=dist,
                        y=(-1+2*wall_i)*(1+2*random.random()),
                        z=self.wall_z_size/2+self.shift_z,
                        roll=0.,
                        pitch=0.,
                        yaw=random.randint(0, 1) * math.pi / 2,
                        box_x=self.wall_x_size,
                        box_y=self.wall_y_size,
                        box_z=self.wall_z_size
                    )
                )
        self.walls = walls
        self.exist = True

class GroundObstacles(Group):

    def __init__(self, wall_x_size, wall_y_size, wall_z_size):
        super().__init__("ground_obstacles")
        self.wall_z_size = wall_z_size
        self.wall_x_size = wall_x_size
        self.wall_y_size = wall_y_size
        self.exist = False

    def generate(self):
        walls = []
        for dist_i in range(4):
            dist = -3.0 - dist_i * 2
            for wall_i in range(2):
                walls.append(
                    Box(
                        name="wall_"+str(dist_i) + "_" + str(wall_i),
                        x=dist,
                        y=(-1+2*wall_i)*(1+2*random.random()),
                        z=self.wall_z_size/2,
                        roll=0.,
                        pitch=0.,
                        yaw=random.randint(0, 1) * math.pi / 2,
                        box_x=self.wall_x_size,
                        box_y=self.wall_y_size,
                        box_z=self.wall_z_size
                    )
                )
        self.walls = walls
        self.exist = True

class StairFloor(Group):

    min_step_height = 0.17  # 0.23 0.21 0.18  0.15
    min_step_length = 0.35
    max_step_height = 0.28  # 0.3 0.28 0.25 0.22
    max_step_length = 0.52
    length_ground = 12.0
    floor_thickness = 0.01
    width_ground = 10.
    width_stair = 2.
    standard_height = 2.0

    def __init__(self):
        super().__init__("stair_floor")
        self.shift_x = 0.
        self.shift_z = 0.

        self.step_height = 0.
        self.step_length = 0.
        self.step_n = 0.
        self.exist = False

    def generate(self):
        # steps
        step_n = random.randint(5, 10)
        length = StairFloor.min_step_length + random.random() * (StairFloor.max_step_length - StairFloor.min_step_length)
        height = StairFloor.min_step_height + random.random() * (StairFloor.max_step_height - StairFloor.min_step_height)
        print("Generating staircase of parameters:", length, height)
        self.step_height = height
        self.step_length = length
        self.step_n = step_n
        self.exist = True

        self.steps = []
        for i in range(step_n):
            self.steps.append(
                Box(
                    name="step_"+str(i),
                    x=length / 2 + length * i,
                    y=0.,
                    z=height / 2 + height * i,
                    roll=0.,
                    pitch=0.,
                    yaw=0.,
                    box_x=length,
                    box_y=2.,
                    box_z=height
                )
            )
        self.shift_x = length * step_n
        self.shift_z = height * step_n
        # floor
        self.floor = Box(
            name="floor",
            x=StairFloor.length_ground/2+self.shift_x,
            y=0.,
            z=self.shift_z - StairFloor.floor_thickness/2,
            roll=0.,
            pitch=0.,
            yaw=0.,
            box_x=StairFloor.length_ground,
            box_y=StairFloor.width_ground,
            box_z=StairFloor.floor_thickness/2
        )
        # walls
        self.walls = [
            Box(
                name="back",
                x=StairFloor.length_ground + self.shift_x,
                y=0.,
                z=self.shift_z + StairFloor.standard_height / 2,
                roll=0.,
                pitch=0.,
                yaw=0,
                box_x=StairFloor.floor_thickness,
                box_y=StairFloor.width_ground,
                box_z=StairFloor.standard_height
            ),
            Box(
                name="right",
                x=StairFloor.length_ground/2 + self.shift_x,
                y=-StairFloor.width_ground/2,
                z=self.shift_z + StairFloor.standard_height / 2,
                roll=0.,
                pitch=0.,
                yaw=math.pi/2,
                box_x=StairFloor.floor_thickness,
                box_y=StairFloor.length_ground,
                box_z=StairFloor.standard_height
            ),
            Box(
                name="left",
                x=StairFloor.length_ground / 2 + self.shift_x,
                y=StairFloor.width_ground / 2,
                z=self.shift_z + StairFloor.standard_height / 2,
                roll=0.,
                pitch=0.,
                yaw=math.pi / 2,
                box_x=StairFloor.floor_thickness,
                box_y=StairFloor.length_ground,
                box_z=StairFloor.standard_height
            ),
            Box(
                name="front_right",
                x=self.shift_x,
                y=-StairFloor.width_ground / 2 + (StairFloor.width_ground/2 - StairFloor.width_stair / 2) / 2 ,
                z=self.shift_z + StairFloor.standard_height / 2,
                roll=0.,
                pitch=0.,
                yaw=0,
                box_x=StairFloor.floor_thickness,
                box_y=StairFloor.width_ground/2 - StairFloor.width_stair / 2,
                box_z=StairFloor.standard_height
            ),
            Box(
                name="front_left",
                x=self.shift_x,
                y=StairFloor.width_ground / 2 - (StairFloor.width_ground / 2 - StairFloor.width_stair / 2) / 2,
                z=self.shift_z + StairFloor.standard_height / 2,
                roll=0.,
                pitch=0.,
                yaw=0,
                box_x=StairFloor.floor_thickness,
                box_y=StairFloor.width_ground / 2 - StairFloor.width_stair / 2,
                box_z=StairFloor.standard_height
            ),
            Box(
                name="side_left",
                x=self.shift_x/2,
                y=StairFloor.width_stair / 2,
                z=self.shift_z/2 + StairFloor.standard_height / 2,
                roll=0.,
                pitch=0.,
                yaw=math.pi/2,
                box_x=StairFloor.floor_thickness,
                box_y=self.shift_x,
                box_z=StairFloor.standard_height + self.shift_z
            ),
            Box(
                name="side_right",
                x=self.shift_x / 2,
                y=-StairFloor.width_stair / 2,
                z=self.shift_z / 2 + StairFloor.standard_height / 2,
                roll=0.,
                pitch=0.,
                yaw=math.pi / 2,
                box_x=StairFloor.floor_thickness,
                box_y=self.shift_x,
                box_z=StairFloor.standard_height + self.shift_z
            ),
        ]


if __name__ == "__main__":
    import render
    e = Env()
    e.ground_obstacles.generate()
    render.apply(e.ground_obstacles)
    e.stair_floor.generate()
    render.apply(e.stair_floor)
    e.floor_obstacles.shift_x = e.stair_floor.shift_x
    e.floor_obstacles.shift_z = e.stair_floor.shift_z
    e.floor_obstacles.generate()
    render.apply(e.floor_obstacles)
