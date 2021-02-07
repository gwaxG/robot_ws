#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
from learning.srv import ConfService, ConfServiceResponse, ConfServiceRequest
from learning.srv import TaskService, TaskServiceResponse, TaskServiceRequest

class RemoteService:

    def __init__(self):
        rospy.init_node("task_service")
        s = rospy.Service('add_task', TaskService, self.add_task)
        s = rospy.Service('get_conf', ConfService, self.get_conf)
        rospy.spin()
        
    def add_task(self, req):
        json_body = json.loads(req.task)
        print("Starting {}".format(json_body['name']))
        return TaskServiceResponse(success=True, err="")
        
    def get_conf(self, req):
        config = ""
        with open("experiment_config.json", "r") as f:
            config = " ".join(f.readlines())
        return ConfServiceResponse(conf=config)
    
if __name__ == "__main__":
    RemoteService()
