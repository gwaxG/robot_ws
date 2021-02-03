# Simulation package

- - -
## Description
This package provides Gazebo utilities.
## Environment generation
Launch file `simulation.launch` starts simulation with basic immovable walls.

The sub-package `env_generation` provides services for handling environment (re)generation through the node `env_gen_services.py`.

The service `env_gen` works with navigation like `/action/model_name` where the action could be `generate` or `delete`.
Supported models are `ground_obstacles`, `stair_floor` and `floor_obstacles`. 
The "stair_floor" model relates to the staircase with the floor.

The service `stair_info` returns staircase length, height and number of steps if the staircas exists which is indicated by the attribute `exist`.

## Robot (re)spawning
Launch file `spawn.launch` spawns the Jaguar robot in front of the staircase one meter away.
This sub-package provides the service `spawn` to actively respawn the robot.
The navigation pattern is `/place/task/random` where:
* the place is `ground` or `floor`;
* the task is `flat` or `traversal`;
* random is `0` or `1`.


