#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from utils.base import Base


# Basic example of launch file usage.
class Learner(Base):
    def __init__(self):
        # self.log to print with datetime call
        super(Learner, self).__init__()

    def learn(self):
        cnt = 0
        while True:
            self.log("Iteration " + str(cnt))
            self.rospy.sleep(1)
            cnt += 1
            if cnt > 2:
                self.log("done")
                break


if __name__ == "__main__":
    Learner().learn()
