#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This node calculates safety estimations both in simulation and real world
such as normalized energy stability margin (NESM) and stability margin (SM).
The monitor node subscribes to results of this one and forms the negative return.
During rollouts in the real world, this node evaluates safety measures of the robot.

Still, it is IMPORTANT to remember that an exterior node like monitor or stair_detector
has to trigger the update about the actual traversed staircase.

PS Mostly, the code is borrowed from the previous version of environment.
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
        # ROS communication
        # UpdateStairInfo trigger service
        # called if it is necessary to update information about staircase
        _ = rospy.Service('stair_info_update_safety', Trigger, self.trigger_stair_info_update)
        # StairInfo proxy service
        # to call the StairInfo service either in simulation or reality
        self.stair_info_proxy = rospy.ServiceProxy('stair_info', StairInfo)
        self.stair_info = None
        # /robot/state subscriber
        # subscribe to the control package to know configuration of the robot
        rospy.Subscriber("/robot/state", State, self.update_robot_state)
        self.robot_state = State()
        # /odom subscriber
        # dead reckoning position in the env. (absolutely precise in simulation)
        rospy.Subscriber("/odom", Odometry, self.update_robot_position)
        # /imu subscriber
        # IMU data from simulation or Jaguar
        rospy.Subscriber("/imu", Imu, self.update_imu)
        # Publishers
        self.safety_nesm = rospy.Publisher("/safety/nesm", Float32, queue_size=1)
        self.safety_sm = rospy.Publisher("/safety/sm", Float32, queue_size=1)
        self.safety_angular = rospy.Publisher("/safety/angular", Float32, queue_size=1)
        # TF listener
        self.listener = tf.TransformListener()
        # Class related objects
        self.mass = {
            'arm_1': 1.0,
            'arm_2': 0.5,
            'arm_3': 0.5,
            'ball': rospy.get_param("ball_mass"),
            'base': 18.0,
        }
        self.sign = lambda x: x/abs(x) if x != 0 else 1
        self.surface_equation = None
        self.orientation = None
        self.last_centroid_height = 0
        self.ax_f = 0
        self.ay_f = 0
        self.az_f = 0
        rospy.spin()

    def update_imu(self, msg):
        self.estimate_safety_angular(msg)
        self.calculate_safety_deviation()

    def update_robot_state(self, msg):
        self.robot_state = msg
        self.calculate_safety_deviation()

    def trigger_stair_info_update(self, req):
        self.stair_info = self.stair_info_proxy.call(StairInfoRequest())
        self.update_surface_equation()
        return TriggerResponse(success=True, message="")

    def update_surface_equation(self):
        """
        Get equation of the surface.
        :return:
        """
        # Get three points corresponding to staircase edges in the map frame
        p = []
        for i in range(3):
            (trans, rot) = self.listener.lookupTransform("map", "p"+str(i), rospy.Time())
            p.append(np.array(trans))
        # These two vectors are in the plane
        v1 = p[2] - p[0]
        v2 = p[1] - p[0]
        # the cross product is a vector normal to the plane
        cp = np.cross(v1, v2)
        a, b, c = cp
        # This evaluates a * x3 + b * y3 + c * z3 which equals d
        d = np.dot(cp, p[2])
        self.surface_equation = [a, b, c, d]

    def calculate_safety_deviation_(self):
        pass

    def calculate_safety_deviation_(self):
        """
        Calculate center of mass in the frame of base
        geometrical centroid in the case of arm usage.
        :return:
        """
        if self.stair_info is None:
            return None
        r, p, y = self.orientation
        a_ = self.robot_state.arm_joint1
        b_ = self.robot_state.arm_joint2
        a = 3.14/2 - a_
        b = 3.14/2 - b_
        cs_rot = -a_
        shift_x = 0.123
        shift_y = 0.18
        length_arm_1 = 0.3
        length_arm_2 = 0.3
        r1 = length_arm_1 / 2
        r2 = length_arm_2 / 2

        # Arm 1
        p1 = [
            shift_x + np.cos(a) * r1,
            shift_y + np.sin(a) * r1
        ]
        # Arm 2
        p2 = [
            shift_x + np.cos(a) * 2 * r1,
            shift_y + np.sin(a) * 2 * r1
        ]
        # Arm 3 in rotated coordinate system
        p3_ = [
            np.cos(b) * r2,
            np.sin(b) * r2
        ]
        # Arm 4 in rotated coordinate system
        p4_ = [
            np.cos(b) * r2 * 2,
            np.sin(b) * r2 * 2
        ]
        # Arm 3 in centroid's coordinate system
        p3 = [
            p2[0] + np.cos(cs_rot) * p3_[0] - np.sin(cs_rot) * p3_[1],
            p2[1] + np.sin(cs_rot) * p3_[0] + np.cos(cs_rot) * p3_[1],
        ]
        # Arm 4 in centroid's coordinate system
        p4 = [
            p2[0] + np.cos(cs_rot) * p4_[0] - np.sin(cs_rot) * p4_[1],
            p2[1] + np.sin(cs_rot) * p4_[0] + np.cos(cs_rot) * p4_[1],
        ]
        COMx = p1[0] * self.mass['arm_1']
        COMx += p2[0] * self.mass['arm_2']
        COMx += p3[0] * self.mass['arm_3']
        COMx += p4[0] * self.mass['ball']
        COMx /= np.sum(list(self.mass.values()))
        COMy = p1[1] * self.mass['arm_1']
        COMy += p2[1] * self.mass['arm_2']
        COMy += p3[1] * self.mass['arm_3']
        COMy += p4[1] * self.mass['ball']
        COMy /= np.sum(list(self.mass.values()))

        COMx_ = COMx * np.cos(r) - COMy * np.sin(r)
        COMy_ = self.last_centroid_height + COMx * np.sin(r) + COMy * np.cos(r)
        base_half_length = 0.32
        deviation = COMx_ - np.tan(r) * COMy_
        COMy_ -= 0.085

        if abs(deviation) > base_half_length:
            if deviation > 0:
                deviation = base_half_length
            else:
                deviation = -base_half_length

        self.safety_sm.publish(Float32(data=deviation))
        self.safety_nesm.publish(Float32(data=COMy_))

    def estimate_safety_angular(self, msg):
        if self.stair_info is None:
            return None
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        rpy = tf.transformations.euler_from_quaternion([x, y, z, w])
        roll = round(-rpy[1], 2)
        pitch = round(rpy[0], 2)
        yaw = round(rpy[2], 2)
        self.orientation = [roll, pitch, yaw]
        g = 9.81  # [m/s^2]
        gz = g * np.cos(roll)
        gx = g * np.sin(roll) * np.cos(yaw)
        gy = g * np.sin(roll) * np.sin(yaw)

        ax = msg.linear_acceleration.x - gx * self.sign(roll)
        ay = msg.linear_acceleration.y - gy
        az = msg.linear_acceleration.z - gz
        e = 0.9
        self.ax_f = (1 - e) * ax + e * self.ax_f
        self.ay_f = (1 - e) * ay + e * self.ay_f
        self.az_f = (1 - e) * az + e * self.az_f

        accel = (msg.linear_acceleration.x) ** 2 + (msg.linear_acceleration.z) ** 2
        accel = accel ** 0.5
        if accel < 0.1:
            accel = 0.0

        angular_velocity_y = abs(msg.angular_velocity.y)
        if angular_velocity_y <= 0.15:
            angular_velocity_y = 0.0
        pen = angular_velocity_y
        self.safety_angular.publish(Float32(data=pen))

    def update_robot_position(self, msg):
        """
        Distance from the base link to the stair surface.
        Base link is located at the robot's bottom, centroid at the center.
        :param msg:
        :return:
        """
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        if self.surface_equation is not None:
            a, b, c, d = self.surface_equation
            dist = abs(a * x + b * y + c * z + d) / (a ** 2 + b ** 2 + c ** 2) ** .5
            surface_z = (-d - a * x) / c
            sign = 1 if surface_z < z else -1
            dist *= sign
            l = self.stair_info.length
            h = self.stair_info.height
            n = self.stair_info.number
            if x > 1 and x < 1.0 + l * n:
                self.last_centroid_height = dist
            elif x < 1:
                self.last_centroid_height = z
            elif x > 1.0 + l * n:
                height = z - h * n
                self.last_centroid_height = height


if __name__ == '__main__':
    Safety()
