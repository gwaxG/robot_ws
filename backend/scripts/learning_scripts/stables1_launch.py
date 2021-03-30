#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from policies import *
import os
import argparse
from stable_baselines.common.vec_env import DummyVecEnv as DummyVecEnv_sb1
from stable_baselines import PPO2 as PPO_sb1
import rospy
from stable_baselines.common.policies import MlpPolicy as PPO2MlpPolicy
import json
from utils.base import Base
from gym_training.envs.training_env import TrainingEnv

class Learner(Base):
    def __init__(self):
        """
        Supported algs
        SACsb1, SACsb3
        PPO2sb1, PPOsb3
        TD3sb1
        """
        self.prms = self.load_prms(__file__.replace(".py", ".json"))
        super(Learner, self).__init__(self.prms)
        kwargs = {
            'experiment_series': self.prms['experiment_series'],
            'experiment': self.prms['experiment'],
            'arm': self.prms['arm'],
            'angular': self.prms['angular'],
            'time_step_limit': self.prms['time_step_limit'],
            'sigma': self.prms['sigma'],
            'task': self.prms['task'],
            'rand': self.prms['rand'],
        }
        # env = gym.make("TrainingEnv-v1", **kwargs)
        env = TrainingEnv(**kwargs)
        self.env = DummyVecEnv_sb1([lambda: env])

        model_parameters = self.prms['model_parameters']
        if self.prms['alg'] == 'PPO2sb1':
            if self.prms['policy_type'] == 'PPO2MlpPolicy':
                self.model = PPO_sb1(PPO2MlpPolicy, self.env, n_steps=128, verbose=1, learning_rate=1e-4,
                                     tensorboard_log=self.log_path)
            if self.prms['policy_type'] == 'CustomMLPsb1':
                self.model = PPO_sb1(
                    CustomMLPsb1, self.env, n_steps=128, verbose=1, learning_rate=1e-4,
                    tensorboard_log=self.log_path,
                    cliprange=float(model_parameters['cliprange']),
                    ent_coef=float(model_parameters['ent_coef'])
                )

    def load_prms(self, fname):
        with open(fname) as f:
            prms = json.load(f)
        return prms

    def train_model(self):
        self.log(f"Learning started! For {type(int(self.prms['total_timesteps']))} of type {type(self.prms['total_timesteps'])}")
        self.model.learn(total_timesteps=int(self.prms['total_timesteps']))
        try:
            self.model.save(self.save_path)
        except Exception as e:
            print("Model was not saved")


if __name__ == '__main__':
    Learner().train_model()
