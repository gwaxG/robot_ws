#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import logging
import os
import sys
import rospy
from base_logger import BaseLogger
from std_msgs.msg import String
import importlib


class Learner(BaseLogger):
    def __init__(self, port):
        super(Learner, self).__init__(fname="log_"+str(port), logger_name="CENTRAL")
        self.port = port
        os.environ["ROS_MASTER_URI"] = "http://localhost:" + str(self.port)
        rospy.init_node("example")
        self.p = rospy.Publisher("endless", String)

    def learn(self):
        cnt = 0
        while not rospy.is_shutdown():
            self.logger.info("Iteration on"+str(self.port))
            self.p.publish(String(data=str(cnt)))
            cnt += 1
            rospy.sleep(1)
            if cnt > 2:
                self.logger.info("done")
                break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', type=int, default=11311, help='ROS MASTER URI port')
    args = parser.parse_args()
    Learner(args.p).learn()
