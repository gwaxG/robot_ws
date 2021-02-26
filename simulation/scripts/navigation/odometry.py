#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance


"""
Simulation odometry based on Gazebo output.
It broadcast base_link in odom frame and publishes odometry.
"""

class Odom:
    def __init__(self):
        rospy.init_node('odom')
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.br = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odometry", Odometry)
        self.seq = 1
        self.sync_time = rospy.get_time()
        rospy.spin()

    def callback(self, msg):
        if rospy.get_time() - self.sync_time < 0.1:
            return None
        else:
            self.sync_time = rospy.get_time()
        names = msg.name
        jaguar_index = names.index("jaguar")
        pose = msg.pose[jaguar_index]
        twist = msg.twist[jaguar_index]
        self.odom_pub.publish(
            Odometry(
                header=Header(
                    seq=self.seq,
                    stamp=rospy.Time.now(),
                    frame_id="odom"
                ),
                child_frame_id="base_link",
                pose=PoseWithCovariance(
                    pose=pose,
                ),
                twist=TwistWithCovariance(
                    twist=twist,
                ),
            )
        )
        self.seq += 1
        self.br.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            rospy.Time.now(),
            "base_link",
            "odom"
        )


if __name__ == "__main__":
    Odom()
