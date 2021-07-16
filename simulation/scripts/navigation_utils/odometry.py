#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from simulation.srv import OdomInfo, OdomInfoResponse, OdomInfoRequest
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from termcolor import colored
import time


"""
Simulation odometry based on Gazebo output.
It broadcast base_link in odom frame and publishes odometry.
"""

class Odom:
    def __init__(self):
        rospy.init_node('odom')
        self.robot_name = ""
        while self.robot_name == "":
            try:
                self.robot_name = rospy.get_param("robot_name")
            except Exception as e:
                print("No robot_name parameter on param server")
                # Sleeping in real time
                time.sleep(0.1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.br = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odometry", Odometry)
        self.seq = 1
        self.sync_time = rospy.get_time()
        # rpy service definition
        s = rospy.Service('odom_info', OdomInfo, self.callback_odom)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        rospy.spin()

    def callback_odom(self, req):
        return OdomInfoResponse(
            result=True,
            err="",
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw
        )

    def callback(self, msg):
        if rospy.get_time() - self.sync_time < 0.1:
            return None
        else:
            self.sync_time = rospy.get_time()
        names = msg.name
        try:
            robot_index = names.index(self.robot_name)
        except ValueError as e:
            print(e)
            print(colored('Model is not in simulation yet', 'red'))
            return None
        pose = msg.pose[robot_index]
        twist = msg.twist[robot_index]
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
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion(
            [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ],
            axes='sxyz'
        )


if __name__ == "__main__":
    Odom()
