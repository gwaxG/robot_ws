from gym.envs.registration import register

register(
    id='TrainingEnv-v1',
    entry_point='gym_training.envs:TrainingEnv',
    kwargs={
        'arm': True,
        'angular': True,
        'sigma': 0.0,
    }
)