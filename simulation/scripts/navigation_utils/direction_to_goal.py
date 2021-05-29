#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from monitor.msg import RolloutAnalytics
from std_msgs.msg import Header
from simulation.msg import DistDirec
import numpy as np
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from termcolor import colored
from simulation.srv import GoalInfo, GoalInfoResponse, GoalInfoRequest
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest

"""
Simulation odometry based on Gazebo output.
It broadcast base_link in odom frame and publishes odometry.
"""


class DirectionToGoal:
    def __init__(self):
        rospy.init_node('direction_to_goal')
        rospy.Subscriber("/odometry", Odometry, self.callback)
        rospy.Subscriber("/rollout/analytics", RolloutAnalytics, self.callback_new_rollout)
        self.goal_exist_call = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        self.listener = tf.TransformListener()
        self.goal_info_call = rospy.ServiceProxy('goal_info', GoalInfo)
        self.goal = None
        self.need_update = True
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher("/direction", DistDirec)
        rospy.spin()

    def callback_new_rollout(self, _):
        self.need_update = True


    def callback(self, msg):
        try:
            (trans, rot) = self.listener.lookupTransform("centroid", "goal", rospy.Time())
        except Exception as e:
            print("In simulation.navigation.direction_to_goal can not check goal-centroid transformation.\n"
                  "Probably, env_gen_services are not running.")
            return None
        try:
            (trans_g, rot_g) = self.listener.lookupTransform("goal", "centroid", rospy.Time())
        except Exception as e:
            print("In simulation.navigation.direction_to_goal can not check goal-centroid transformation.\n"
                  "Probably, env_gen_services are not running.")
            return None
        print("trans_g", trans_g)
        xg, yg, zg = trans
        # Cartesian to spherical coordinate frame
        distance = (trans[0] ** 2 + trans[1] ** 2 + trans[2] ** 2) ** 0.5
        theta = np.arccos(zg/np.sqrt(xg**2 + yg**2 + zg**2))
        phi = np.arcsin(yg/np.sqrt(xg**2+yg**2))
        dist_center_plane = 0.
        self.pub.publish(
            DistDirec(
                distance=distance,
                theta=theta,
                phi=phi,
                dist_center_plane=dist_center_plane
            )
        )
        # print(f"Distance {distance}; angle {heading}")

    def update_goal(self):
        self.goal = self.goal_info_call.call(GoalInfoRequest())


if __name__ == "__main__":
    DirectionToGoal()
