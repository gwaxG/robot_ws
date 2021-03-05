#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from policies import *
import os
import numpy as np
import argparse
from stable_baselines3.common.vec_env import DummyVecEnv as DummyVecEnv_sb3
from stable_baselines.common.vec_env import DummyVecEnv as DummyVecEnv_sb1
from stable_baselines import TD3
from stable_baselines.ddpg.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines.td3.policies import MlpPolicy as TD3sb1MlpPolicy
from stable_baselines import PPO2 as PPO_sb1
from stable_baselines3 import PPO as PPO_sb3
from std_msgs.msg import String
import rospy
from stable_baselines.common.policies import MlpPolicy as PPO2MlpPolicy
from stable_baselines.common.policies import MlpLnLstmPolicy
import json
# from policies.CustomPolicies import *
from stable_baselines.sac.policies import MlpPolicy as SacMlpPolicy
from stable_baselines import SAC
from stable_baselines3 import SAC as SAC3
from stable_baselines3.sac.policies import SACPolicy as SACPolicySB3

class LearningOpenAI:
    def __init__(self, fname="config1.json"):
        """
        Supported algs
        SACsb1, SACsb3
        PPO2sb1, PPOsb3
        TD3sb1

        :param fname: configuration file name
        """
        self.prms = self.load_prms(fname)
        kwargs = {
            'experiment': self.prms['experiment'],
            'arm': self.prms['arm'],
            'angular': self.prms['angular'],
            'sigma': self.prms['sigma'],
            'task': self.prms['task'],
            'rand': self.prms['rand'],
        }
        env = gym.make("TrainingEnv-v1", **kwargs)
        self.env = DummyVecEnv_sb1([lambda: env])

        model_parameters = self.prms['model_parameters']
        if self.prms['alg'] == 'PPO2sb1':
            if self.prms['policy_type'] == 'PPO2MlpPolicy':
                self.model = PPO_sb1(PPO2MlpPolicy, self.env, n_steps=128, verbose=1, learning_rate=1e-4,
                                     tensorboard_log=self.prms['log_path'])
            if self.prms['policy_type'] == 'CustomMLPsb1':
                self.model = PPO_sb1(
                    CustomMLPsb1, self.env, n_steps=128, verbose=1, learning_rate=1e-4,
                    tensorboard_log=self.prms['log_path'],
                    cliprange=float(model_parameters['cliprange']),
                    ent_coef=float(model_parameters['ent_coef'])
                )

    def load_prms(self, fname):
        with open(fname) as f:
            prms = json.load(f)
        return prms

    def train_model(self):
        self.model.learn(total_timesteps=self.prms['total_timesteps'])
        try:
            self.model.save(self.prms['save_path'])
        except Exception as e:
            print("Model was not saved")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-conf', type=str, default="config1.json", help='configuration file path')
    args = parser.parse_args()

    LearningOpenAI(fname=args.conf).train_model()
