#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import rospy
from utils.stream_logger import StreamLogger
import datetime
from std_msgs.msg import String


class Learner:
    def __init__(self, port):
        self.stream_logger = StreamLogger(port)
        self.port = port
        self.log = lambda s: print(datetime.datetime.now(), s)
        os.environ["ROS_MASTER_URI"] = "http://localhost:" + str(self.port)
        rospy.init_node("example")
        self.p = rospy.Publisher("endless", String)

    def learn(self):
        cnt = 0
        while not rospy.is_shutdown():
            self.log("Iteration on"+str(self.port))
            self.p.publish(String(data=str(cnt)))
            cnt += 1
            rospy.sleep(1)
            if cnt > 2:
                self.log("done")
                break

    def __del__(self):
        self.stream_logger.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', type=int, default=11311, help='ROS MASTER URI port')
    args = parser.parse_args()
    Learner(args.p).learn()


