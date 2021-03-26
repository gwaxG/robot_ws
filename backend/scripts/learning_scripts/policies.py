import sys
import numpy as np
from numpy.random import default_rng
import time

import gym
from gym import spaces
import os
from torchvision import transforms

from stable_baselines3 import DQN
from stable_baselines3.dqn import CnnPolicy
from stable_baselines3.common.env_checker import check_env

import torch as th
import torch.nn as nn
import torchvision

from stable_baselines.common.policies import FeedForwardPolicy, register_policy
from stable_baselines3 import PPO, DQN
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from torch.utils.data import DataLoader


class CustomResnetCNN(CnnPolicy):
    """
    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, action_space: gym.spaces.Box, features_dim):
        super(CustomResnetCNN, self).__init__(observation_space, features_dim)
        # We assume CxHxW images (channels first)
        # Re-ordering will be done by pre-preprocessing or wrapper
        self.cnn = torchvision.models.resnet18(pretrained=True)
        num_ftrs = self.cnn.fc.in_features
        # self.cnn.fc = nn.Linear(num_ftrs, 2)
        self.device = th.device("cuda:0" if th.cuda.is_available() else "cpu")
        self.cnn = self.cnn.to(self.device)
        with th.no_grad():
            n_flatten = self.cnn(
                th.as_tensor(observation_space.sample()[None]).float()
            ).shape[1]
        self.cnn.fc = nn.Sequential(nn.Linear(n_flatten, num_ftrs), nn.ReLU())

    def forward(self, observations: th.Tensor) -> th.Tensor:
        # Cut off image
        # reshape to from vector to W*H
        # gray to color transform
        # application of ResNet
        # Concat features to the rest of observation vector
        # return
        observations = th.Tensor(observations)
        observations = observations.to(self.device)
        return self.cnn(observations)

class CustomMLPsb1(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomMLPsb1, self).__init__(*args, **kwargs,
                                           net_arch=[dict(pi=[140, 128, 64],
                                                          vf=[140, 128, 64])],
                                           feature_extraction="mlp")