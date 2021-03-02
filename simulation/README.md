# Simulation package

## Description
This package provides Gazebo simulation utilities.

- - -
## Environment generation
`simulation.launch` starts simulation with basic immovable walls and services that manage environment reconfiguration.

The service `env_gen` works with EnvGen.srv:
```
string action
string model
string props
---
bool result
string err
```
where the action could be `generate` or `delete`, supported models are `ground_obstacles`, `stair_floor` and `floor_obstacles`. 
The "stair_floor" model relates to the staircase with the floor.
The field `result` indicates if generation was succesful, if `false`, the field `err` explains why.  
The goal can be spawned through `action=generate/delete`, `model=goal` and `props=task_rand` where task could be ascent, descent or flat and rand is 0 or 1. 


The service `stair_info` operates with StairInfo.srv:
```
---
float64 length
float64 height
float64 number
bool exist
```
where fields `length`, `height` and `number`(number of steps) relates to the staircase parameters, staircase existence is indicated by the `exist` field.

- - -
## Robot (re)spawning

`spawn.launch` initially spawns the Jaguar robot in fornt of the staircase one meter away.
This sub-package provides the service `robot_spawn` to actively respawn the robot at other locations.
RobotSpawn.srv:

```
string place
string task
string random
---
bool result
string err
```

where the place is `ground` or `floor`, the task is `flat`, `ascent` or `descent`, random is `0` or `1`.



