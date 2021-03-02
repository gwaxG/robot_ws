#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from monitor.srv import NewRollout, NewRolloutRequest

if __name__ == '__main__':
    rospy.init_node('test_node')
    r = rospy.ServiceProxy('/rollout/new', NewRollout)
    res = r("test_exp1", 1, "", False, False)
    print(res)