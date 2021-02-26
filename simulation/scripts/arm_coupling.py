#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import time
import json, os
import numpy as np
from control.msg import State
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

'''
This node connects flippers and arm joints to the base
'''


class ArmCoupling:
    def __init__(self):
        rospy.init_node('arm_coupling')
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        arm_1 = 0.0 #  rospy.get_param("arm1_init")
        arm_2 = 0.0 #  rospy.get_param("arm2_init")
        self.arm_positions = {
            "arm_1": [0.123, -0.08, 0.27],
            "arm_2": [-0.055, -0.035, 0.33]
        }
        self.arm_rotations = {
            "arm_1": [-arm_1, 0., 1.5708],
            "arm_2": [0., arm_2, 1.5708]
        }
        rospy.Subscriber("/robot/state", State, self.update_arm_configuration, queue_size=1)


    def update_arm_configuration(self, msg):
        self.arm_rotations['arm_1'][0] = msg.arm_joint1
        self.arm_rotations['arm_2'][1] = msg.arm_joint2

    def broadcast(self, trans, quaternion, child, parent):
        self.br.sendTransform(
            trans,
            quaternion,
            rospy.Time.now(),
            child,
            parent
        )

    def run(self):
        while not rospy.is_shutdown():
            quaternion = tf.transformations.quaternion_from_euler(*self.arm_rotations['arm_1'])
            self.broadcast(self.arm_positions['arm_1'], quaternion, 'arm_1', 'base_link')
            quaternion = tf.transformations.quaternion_from_euler(*self.arm_rotations['arm_2'])
            self.broadcast(self.arm_positions['arm_2'], quaternion, 'arm_2', 'arm_1')
            rospy.sleep(0.1)


if __name__ == '__main__':
    ArmCoupling().run()