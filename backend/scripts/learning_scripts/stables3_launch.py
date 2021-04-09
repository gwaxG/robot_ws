#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from policy import *
import json
from utils.base import Base
from gym_training.envs.training_env import TrainingEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv



class Learner(Base):
    def __init__(self):
        self.prms = self.load_prms(__file__.replace(".py", ".json"))
        super(Learner, self).__init__(self.prms)
        kwargs = {
            'experiment_series': self.prms['experiment_series'],
            'experiment': self.prms['experiment'],
            'arm': self.prms['arm'],
            'angular': self.prms['angular'],
            'penalty_deviation': self.prms['penalty_deviation'],
            'penalty_angular': self.prms['penalty_angular'],
            'time_step_limit': self.prms['time_step_limit'],
            'sigma': self.prms['sigma'],
            'task': self.prms['task'],
            'rand': self.prms['rand'],
        }

        # env = gym.make("TrainingEnv-v1", **kwargs)
        env = TrainingEnv(**kwargs)
        self.env = DummyVecEnv_sb1([lambda: env])

        if self.prms['policy_type'] == "PpoMlpLstm":
                self.model = PPO("CnnPolicy", "Tracked-v1", n_steps=128, verbose=1, learning_rate=1e-4,
                    tensorboard_log=self.log_path,
                    cliprange=float(model_parameters['cliprange']),
                    ent_coef=float(model_parameters['ent_coef'])

        model_parameters = self.prms['model_parameters']

        if self.prms['alg'] == 'TD3':
            if self.prms['policy_type'] == 'Td3MlpLstm':
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
