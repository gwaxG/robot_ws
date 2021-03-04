#!/usr/bin/env python
# -*- coding: utf-8 -*-

import gym
import rospy
import numpy as np
from gym import spaces
from control.msg import State
from monitor.msg import StepReturn, StepReturnRequest
from simulation.srv import RobotSpawn, RobotSpawnResponse, RobotSpawnRequest
from simulation.srv import EnvGen, EnvGenResponse, EnvGenRequest
from perception.msg import BeamMsg
from std_msgs.msg import String

class TrainingEnv(gym.Env):
    ACTION_TIME = 1.0
    def __init__(self, **kwargs):
        self.arm_is_used = kwargs['arm']
        self.angular_is_used = kwargs['angular']
        self.sigma = kwargs['sigma']
        self.task = kwargs['task']
        self.obstacle = self.replace_task_obstacle(self.task)
        self.rand = "1" if kwargs['rand'] else "0"
        #####
        # linear and angular are set, other fields are incremental
        self.action = State()
        # construct action fields
        self.active_action_fields = self.build_action_fields()
        # Publishers
        self.pub_robot_cmd = rospy.Publisher("robot_cmd", State)
        # Subscirbers
        rospy.Subscriber("/robot/state", State, self.update_state)
        self.robot_state = State()
        rospy.Subscriber("/features", BeamMsg, self.update_features)
        self.features = BeamMsg()
        # Service callers
        self.step_return = rospy.ServiceProxy("/rollout/step_return", StepReturn)
        self.robot_spawner = rospy.ServiceProxy('robot_spawn', RobotSpawn)
        self.enb_gen = rospy.ServiceProxy('env_gen', EnvGen)
        # Spaces
        self.action_space, self.observation_space = self.get_spaces()
        #
        self.enb_gen.call(
            EnvGenRequest(
                action="generate",
                model="goal",
                props=self.task + "_" + self.rand,
            )
        )

    def get_spaces(self):
        ANGLE =np.pi / 4
        dMA = np.pi / 10.0
        VEL = 1.0
        amin = []
        amax = []
        omin = []
        omax = []
        fmin = []
        fmax = []

        for k, v in self.active_action_fields.items():
            if v == "linear" or v == "angular":
                amin.append(-Vel)
                amax.append(Vel)
                omin.append(-VEL)
                omax.append(VEL)
            elif v == "front_flippers" or v == "rear_flippers" or v == "arm_joint1" or v == "arm_joint2":
                amin.append(-dMA)
                amax.append(dMA)
                omin.append(-ANGLE)
                omax.append(ANGLE)

        height = rospy.get_param("feature_height")
        width = rospy.get_param("feature_width")
        fmin = [0.0 for i in range(height+width)]
        fmax = [3.0 for i in range(height+width)]
        omin += fmin
        omax += fmax

        aspace = spaces.Box(np.array(amin), np.array(amax))
        ospace = spaces.Box(np.array(omin), np.array(omax))
        return aspace, ospace

    def build_action_fields(self):
        d = {0: 'linear'}
        index = 1
        if self.arm_is_used:
            d[index] = 'angular'
            index += 1
        d[index] = 'front_flippers'
        index += 1
        d[index] = 'rear_flippers'
        index += 1
        if self.arm_is_used:
            d[index] = 'arm_joint1'
            index += 1
            d[index] = 'arm_joint2'
            index += 1
        return d

    def replace_task_obstacle(self, task):
        obstacle = ""
        if self.task == "flat":
            osbtacle = "ground_obstacles"
        elif self.task == "ascent" or self.task == "descent":
            obstacle = "stair_floor"
        else:
            raise(NotImplementedError())
        return obstacle

    def update_features(self, msg):
        self.features = msg

    def update_state(self, msg):
        self.robot_state = msg

    def update_action(self, action):
        """
        Possible configurations:
        action => [linear, angular, front_flippers, rear_flippers, arm_joint1, arm_joint2]
        action => [linear, front_flippers, rear_flippers, arm_joint1, arm_joint2]
        action => [linear, angular, front_flippers, rear_flippers]
        action => [linear, front_flippers, rear_flippers]
        :param action:
        :return:
        """
        for i, action_value in enumerate(action):
            setattr(self.action, self.active_action_fields[i], action_value)

    def get_transformed_state(self):
        """
        state => [robot state, vertical, horizontal]
        :return:
        """
        state = []
        for k, v in self.active_action_fields.items():
            state.append(getattr(self.robot_state), v, 0.)
        state += self.features.vertical.data
        state += self.features.horizontal.data
        return state

    def regenerate_obstacles(self):
        self.enb_gen.call(
            EnvGenRequest(
                action="delete",
                model=self.obstacle,
                props=self.task + "_" + self.rand,
            )
        )
        self.enb_gen.call(
            EnvGenRequest(
                action="generate",
                model=self.obstacle,
                props=self.task + "_" + self.rand,
            )
        )

    def respawn_robot(self):
        if self.task == "ascent" or self.task == "flat":
            ground = "ground"
        if self.task == "descent":
            ground = "floor"
        self.robot_spawn.call(RobotSpawnRequest(
            place=ground,
            task=self.task,
            rand=self.rand,
        ))

    def return_robot_to_initial_state(self):
        self.pub_robot_cmd.publish(
            State(
                front_flipeprs = -self.robot_state.front_flippers,
                rear_flippers=-self.robot_state.rear_flippers,
                arm_joint1=-self.robot_state.arm_joint1,
                arm_joint2=-self.robot_state.arm_joint2,
            )
        )

    def reset(self, goal=""):
        # self.enb_gen.call(EnvGenRequest(action="generate",model="goal",props=self.task + "_" + self.rand,))
        self.regenerate_obstacles()
        self.respawn_robot()
        self.return_robot_to_initial_state()
        return self.get_transformed_state()

    def step(self, action):
        self.update_action(action)
        self.pub_robot_cmd.publish(self.action)
        rospy.sleep(TrainingEnv.ACTION_TIME)
        step_return = self.step_return.call(StepReturnReq())
        reward = step_return.reward
        done = step_return.done
        return self.get_transformed_state(), reward, done, {}

    def render(self, mode='human'):
        pass