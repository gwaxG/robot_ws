#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from simulation.srv import RobotSpawn, RobotSpawnResponse
from spawner import Spawner

class RobotSpawnService:
    def __init__(self):
        rospy.init_node('robot_spawn_service')
        s = rospy.Service('robot_spawn', RobotSpawn, self.router)
        self.spawner = Spawner()
        rospy.spin()


    def router(self, req):
        """
        Basic logic of the spawn process using method chaining.
        :param req:
        :return:
        """
        try:
            self.spawner.init().set_place(req.place).set_task(req.task).set_randomness(req.rand, req.task).spawn()
        except Exception as e:
            print(e)
            return RobotSpawnResponse(result=False, err="Failed to spawn. Error on the service side.")
        return RobotSpawnResponse(result=True, err="")


if __name__ == "__main__":
    RobotSpawnService()