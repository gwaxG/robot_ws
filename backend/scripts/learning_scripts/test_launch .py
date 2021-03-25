#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

rospy.init_node("test_endless_runner")
parser = argparse.ArgumentParser()

parser.add_argument('-rmu', type=str, default="http://localhost:11311", help='ROS MASTER URI')
args = parser.parse_args()
os.environ["ROS_MASTER_URI"] = args.rmu

p = rospy.Publisher("endless", String)

cnt = 0
while not rospy.is_shutdown():
    p.publish(String(data=str(cnt)))
    cnt += 1
    rospy.sleep(1)
    if cnt > 15:
        break