#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This node calculates deviation and pitch angular velocity estimations both in simulation and real world.
The monitor node has to subscribe to results of this node and form the negative return.
During rollouts in the real world, this node evaluates safety measures of the robot.

Still, it is IMPORTANT to remember that an exterior node like monitor or stair_detector
has to trigger the update about the actual being traversed staircase.
"""

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from simulation.srv import StairInfo, StairInfoRequest
from control.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

import tf
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np


class Safety:
    def __init__(self):
        rospy.init_node('safety')
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.mass = {
            'arm1': 1.0,
            'arm2': 0.5,
            'arm3': 0.5,
            'ball': rospy.get_param("ball_mass"),
            'cent': 18.0,
        }
        self.mass["sum"] = np.sum(list(self.mass.values()))
        self.zero_rot = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.pub_dev = rospy.Publisher("/safety/relative_deviation", Float32)
        self.pub_angular = rospy.Publisher("/safety/angular", Float32)
        rospy.Subscriber("/imu", Imu, self.update_imu)
        while not rospy.is_shutdown():
            position = self.define_position()
            if position is None:
                continue
            self.broadcast_centroid_projection(position)
            self.broadcast_cog()
            self.broadcast_cog_projections(position)
            rospy.sleep(0.1)

    def update_imu(self, msg):
        angular_velocity_y = abs(msg.angular_velocity.y)
        angular_velocity_y = angular_velocity_y if angular_velocity_y > 0.15 else 0.
        self.pub_angular.publish(Float32(data=angular_velocity_y))

    def define_position(self):
        try:
            (d, _) = self.listener.lookupTransform('/p_cent', '/centroid', rospy.Time(0))
            (u, _) = self.listener.lookupTransform('/p_cent_up', '/centroid', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        x1 = d[0]
        x2 = u[0]
        if x1 < 0 and x2 < 0:
            return "ground"
        elif x1 > 0 and x2 < 0:
            return "stair"
        elif x1 > 0 and x2 > 0:
            return "floor"

    def broadcast_centroid_projection(self, position):
        # listen to the centroid frame in the p_cent_inc frame
        try:
            if position == "stair":
                (trans, rot) = self.listener.lookupTransform('/p_cent_inc', '/centroid', rospy.Time(0))
            elif position == "ground":
                (trans, rot) = self.listener.lookupTransform('/map', '/centroid', rospy.Time(0))
            elif position == "floor":
                (trans, rot) = self.listener.lookupTransform('/p_cent_up', '/centroid', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        # 0.085 the half of the base height
        trans[2] = 0.085
        if position == "stair":
            self.broadcast(trans, self.zero_rot, "O", "p_cent_inc")
        elif position == "ground":
            self.broadcast(trans, self.zero_rot, "O", "map")
        elif position == "floor":
            self.broadcast(trans, self.zero_rot, "O", "p_cent_up")

    def broadcast_cog(self):
        try:
            (arm1, rot) = self.listener.lookupTransform('/centroid', '/arm_1_mass', rospy.Time(0))
            (arm2, rot) = self.listener.lookupTransform('/centroid', '/arm_2_mass', rospy.Time(0))
            (arm3, rot) = self.listener.lookupTransform('/centroid', '/arm_3_mass', rospy.Time(0))
            (ball, rot) = self.listener.lookupTransform('/centroid', '/ball', rospy.Time(0))
            (cent, rot) = self.listener.lookupTransform('/centroid', '/centroid_mass', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        cog = [
            (
                arm1[i] * self.mass["arm1"] +
                arm2[i] * self.mass["arm2"] +
                arm3[i] * self.mass["arm3"] +
                ball[i] * self.mass["ball"] +
                cent[i] * self.mass["cent"]
            ) / self.mass["sum"] for i in range(3)
        ]
        self.broadcast(cog, self.zero_rot, "cog", "centroid")

    def broadcast_cog_projections(self, position):
        try:
            (cog, rot) = self.listener.lookupTransform('O', 'cog', rospy.Time(0))
            (_, inc) = self.listener.lookupTransform('p_cent', 'p_cent_inc', rospy.Time(0))
            inc = tf.transformations.euler_from_quaternion(inc)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        cy = [0, cog[1], cog[2]]
        self.broadcast(cy, self.zero_rot, "Cy", "O")
        if position == "stair":
            x = cog[0] - np.tan(abs(inc[1]))*cog[2]
        else:
            x = cog[0]
        cx = [x, cog[1], 0]
        self.broadcast(cx, self.zero_rot, "Cx", "O")
        min_deviation = 0.08
        max_deviation = 0.3
        deviation = (cy[2]**2 + cx[0]**2) ** 0.5
        relative_deviation = (deviation - min_deviation) / (max_deviation - min_deviation)
        if relative_deviation < 0:
            relative_deviation = 0.
        elif relative_deviation > 1.:
            relative_deviation = 1.
        if position == "stair":
            self.pub_dev.publish(Float32(data=relative_deviation))
        else:
            self.pub_dev.publish(Float32(data=0.))

    def broadcast(self, trans, quaternion, child, parent):
        quaternion = list(quaternion)
        self.br.sendTransform(
            trans,
            quaternion,
            rospy.Time.now(),
            child,
            parent
        )


if __name__ == '__main__':
    Safety()
