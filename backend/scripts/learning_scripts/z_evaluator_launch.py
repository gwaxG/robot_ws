#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import json
import numpy as np
from utils.base import Base
from gym_training.envs.training_env import TrainingEnv
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3.sac.policies import SACPolicy
from monitor.srv import GuidanceInfoRequest
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

class Learner(Base):
    def __init__(self):
        with open(__file__.replace(".py", ".json")) as f:
            prms = json.load(f)
        super(Learner, self).__init__(prms)
        kwargs = {
            'experiment_series': prms['experiment_series'],
            'experiment': prms['experiment']+"_evaluation",
            'arm': prms['arm'],
            'angular': prms['angular'],
            'penalty_deviation': prms['penalty_deviation'],
            'penalty_angular': prms['penalty_angular'],
            'time_step_limit': prms['time_step_limit'],
            'sigma': prms['sigma'],
            'task': prms['task'],
            'rand': prms['rand'],
            'env_type': "eval",
            'complexity': "full"
        }
        self.env = DummyVecEnv([lambda: TrainingEnv(**kwargs)])

        policies = {
            "default": "MlpPolicy",
            "sac_default": "MlpPolicy",
            "ppo_default": "MlpPolicy",
            "td3_default": "MlpPolicy",
        }
        n_actions = self.env.action_space.shape[-1]
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
        models = {
            "SAC": SAC.load(self.save_path, env=self.env),
        }
        self.model = models[prms["alg"]]
        self.prms = prms

    def evaluate_model(self):
        # Enjoy trained agent
        obs = self.env.reset()
        for i in range(self.prms["total_timesteps"]):
            action, _states = self.model.predict(obs, deterministic=True)
            obs, rewards, dones, info = self.env.step(action)
            self.env.render()


if __name__ == '__main__':
    Learner().evaluate_model()
