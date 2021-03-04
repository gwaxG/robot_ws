from gym.envs.registration import register

register(
    id='TrainingEnv-v1',
    entry_point='gym_training.envs:TrainingEnv',
    kwargs={
        'experiment': "not_defined",
        'arm': True,
        'angular': True,
        'sigma': 0.0,
        'task': 'ascent',
        'rand': False,
    }
)