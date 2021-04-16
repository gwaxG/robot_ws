#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import json
import numpy as np
from utils.base import Base
from gym_training.envs.training_env import TrainingEnv
from stable_baselines3 import PPO
from stable_baselines3 import SAC
from stable_baselines3.sac.policies import SACPolicy
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
            'experiment': prms['experiment'],
            'arm': prms['arm'],
            'angular': prms['angular'],
            'penalty_deviation': prms['penalty_deviation'],
            'penalty_angular': prms['penalty_angular'],
            'time_step_limit': prms['time_step_limit'],
            'sigma': prms['sigma'],
            'task': prms['task'],
            'rand': prms['rand'],
        }
        env = DummyVecEnv([lambda: TrainingEnv(**kwargs)])

        policies = {
            "default": "MlpPolicy",
            "sac_default": "MlpPolicy",
            "ppo_default": "MlpPolicy",
            "td3_default": "MlpPolicy",
        }
        model_parameters = prms['model_parameters']
        n_actions = env.action_space.shape[-1]
        action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
        models = {
            "SAC": SAC(
                policies[prms["policy"]],
                env,
                verbose=1,
                train_freq=float(model_parameters["sac_train_freq"]),
                tau=float(model_parameters["sac_tau"]),
                ent_coef=model_parameters["sac_ent_coef"],
                tensorboard_log=self.log_path
            ),
            "PPO": PPO(
                policies[prms["policy"]],
                env,
                verbose=1,
                tensorboard_log=self.log_path
            ),
            "TD3": TD3(
                policies[prms["policy"]],
                env,
                action_noise=action_noise,
                verbose=1,
                tensorboard_log=self.log_path
            ),
        }
        self.model = models[prms["alg"]]
        self.prms = prms

    def train_model(self):
        self.log(f"Learning started! For {type(int(self.prms['total_timesteps']))} of type {type(self.prms['total_timesteps'])}")
        self.model.learn(total_timesteps=int(self.prms['total_timesteps']), log_interval=4)
        try:
            self.model.save(self.save_path)
        except Exception as e:
            print("Model was not saved")

if __name__ == '__main__':
    Learner().train_model()
