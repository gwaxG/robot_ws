#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
The monitor node.
"""

import rospy
import utils
from guidance import Guidance
from std_srvs.srv import Trigger, TriggerResponse
from simulation.srv import StairInfo, StairInfoRequest
from simulation.srv import GoalInfo, GoalInfoRequest, GoalInfoResponse
from simulation.srv import StairInfo, StairInfoRequest, StairInfoResponse
from monitor.srv import NewRollout, NewRolloutRequest, NewRolloutResponse
from monitor.srv import StepReturn, StepReturnRequest, StepReturnResponse
from monitor.msg import RolloutAnalytics
from monitor.srv import GuidanceInfo, GuidanceInfoResponse
from control.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float32

import tf
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np


class Monitor:
    def __init__(self):
        # TODO 1 change backend to accept RolloutAnalytics msg and not String
        rospy.init_node('monitor')
        # Subscribers
        rospy.Subscriber("/robot/state", State, self.callback_robot_state)
        rospy.Subscriber("/safety/relative_deviation", Float32, self.callback_safety_deviation)
        rospy.Subscriber("/safety/angular", Float32, self.callback_safety_angular)
        rospy.Subscriber("/odometry", Odometry, self.callback_odometry)
        # Publishers
        self.pub_rollout_analytics = rospy.Publisher("/rollout/analytics", RolloutAnalytics)
        # Clients
        self.goal_info = rospy.ServiceProxy("goal_info", GoalInfo)
        self.stair_info = rospy.ServiceProxy("stair_info", StairInfo)
        # Services
        s = rospy.Service("/rollout/new", NewRollout, self.callback_new_rollout)
        s = rospy.Service("/rollout/step_return", StepReturn, self.callback_step_return)
        s = rospy.Service("/rollout/start", Trigger, self.callback_start_rollout)
        s = rospy.Service("/guidance/info", GuidanceInfo, self.callback_guidance)
        # Data
        self.robot_state = None
        self.safety_step_deviation = []
        self.safety_step_angular = []
        self.odometry = None
        self.rollout_state = utils.RolloutState()
        self.goal = None
        self.guide = Guidance()
        self.is_guided = False
        self.stair = None
        rospy.spin()

    def callback_guidance(self, _):
        self.is_guided = True
        return GuidanceInfoResponse(
            epsilon=self.guide.epsilon,
            level=self.guide.level,
        )

    def update_goal(self):
        self.goal = self.goal_info.call(GoalInfoRequest())

    def update_stair(self):
        self.stair = self.stair_info.call(StairInfoRequest())

    def callback_start_rollout(self, req):
        self.rollout_state.time_step = 1
        self.update_goal()
        self.update_stair()
        dist = utils.get_distance(self.odometry.pose.pose.position, self.goal)
        self.rollout_state.closest_distance = dist
        self.rollout_state.maximum_distance = dist
        self.rollout_state.started = True
        return TriggerResponse(success=True, message="")

    def callback_new_rollout(self, req):
        self.rollout_state.reset()
        self.rollout_state.exp_series = rospy.get_param("exp_series_name")
        self.rollout_state.set_fields(req)
        self.guide.set_seq(self.rollout_state.seq)
        return NewRolloutResponse(received=True)

    def callback_step_return(self, _):
        reward = self.rollout_state.step_reward
        self.rollout_state.step_reward = 0.
        step_penalty = 0.
        if self.rollout_state.use_penalty_angular:
            step_penalty += np.mean(self.rollout_state.step_angular)
            self.rollout_state.step_angular = []
            self.rollout_state.episode_angular.append(step_penalty)
        if self.rollout_state.use_penalty_deviation:
            step_penalty += np.mean(self.rollout_state.step_deviation)
            self.rollout_state.step_deviation = []
            reward -= step_penalty
            self.rollout_state.episode_deviation.append(step_penalty)
        step_penalty = self.guide.reshape_penalty(step_penalty)
        if "tip" in self.rollout_state.accidents:
            step_penalty = 1 - self.rollout_state.episode_penalty
        reward -= step_penalty
        self.rollout_state.episode_reward += reward
        if self.rollout_state.done:
            self.send_to_backend()
            if self.is_guided:
                self.guide.update(reward)
        self.rollout_state.time_step += 1
        if self.rollout_state.time_step == self.rollout_state.time_step_limit:
            self.rollout_state.done = True
        return StepReturnResponse(reward=reward, done=self.rollout_state.done)

    def send_to_backend(self):
        log = self.guide.log_string if self.guide.log_update else ""
        self.pub_rollout_analytics.publish(
            RolloutAnalytics(
                exp_series=self.rollout_state.exp_series,
                experiment=self.rollout_state.experiment,
                seq=self.rollout_state.seq,
                sensors=self.rollout_state.sensors,
                arm=self.rollout_state.arm,
                angular=self.rollout_state.angular,
                progress=self.rollout_state.progress,
                reward=self.rollout_state.episode_reward,
                angular_m=np.mean(self.rollout_state.episode_angular),
                deviation=np.mean(self.rollout_state.episode_deviation),
                accidents=self.rollout_state.accidents,
                time_steps=self.rollout_state.time_step,
                log=log
            )
        )

    def callback_robot_state(self, msg):
        """
        Update robot state.
        :param msg: State msg
        :return:
        """
        self.robot_state = msg

    def callback_safety_deviation(self, msg):
        self.rollout_state.step_deviation.append(msg.data)

    def callback_safety_angular(self, msg):
        self.rollout_state.step_angular.append(msg.data)

    def callback_odometry(self, msg):
        self.odometry = msg
        if self.rollout_state.done or not self.rollout_state.started:
            return
        # distance check
        dist = utils.get_distance(self.odometry.pose.pose.position, self.goal)
        if dist < self.rollout_state.closest_distance:
            diff = self.rollout_state.closest_distance - dist if dist >= 0.0 else self.rollout_state.closest_distance
            self.rollout_state.progress += diff / self.rollout_state.maximum_distance
            self.rollout_state.step_reward += diff
        if dist < 0.0:
            self.rollout_state.closest_distance = 0.
            self.rollout_state.done = True
        else:
            self.rollout_state.closest_distance = dist
        # tipping over check
        wxyz = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ]
        roll, pitch, _ = tf.transformations.euler_from_quaternion(wxyz)
        accident = False
        if roll > np.pi / 2:
            self.rollout_state.accidents = "Front tipping over"
            accident = True
        elif roll < -np.pi / 2:
            self.rollout_state.accidents = "Rear tipping over"
            accident = True
        if pitch > np.pi / 2:
            self.rollout_state.accidents = "Right tipping over"
            accident = True
        elif pitch < -np.pi / 2:
            self.rollout_state.accidents = "Left tipping over"
            accident = True
        if accident:
            self.rollout_state.done = True


if __name__ == '__main__':
    Monitor()
