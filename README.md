# Robot work space
***
## Description
This work space contains all necessary code for the simulation, control, perception and learning of the Jaguar V4 with Arm robot, as part of the following publication :

@article{AR-Mitriakov-2022,
  author = {A. Mitriakov and P. Papadakis and S. Garlatti},
  title = {An open-source software framework for reinforcement learning-based control of tracked robots in simulated indoor environments},
  journal = {Advanced Robotics},
  volume = {0},
  number = {0},
  pages = {1-14},
  year  = {2022},
  publisher = {Taylor & Francis},
  doi = {10.1080/01691864.2022.2076570},
}

Software was tested on a machine with the following hardware configuration:
* Memory 31,2 GiB
* Processor Intel® Core™ i7-8650U CPU @ 1.90GHz × 8 
* Graphics Intel® UHD Graphics 620 (KBL GT2)
* \>30 GiB HDD/SSD

## Requirements and installation
The user is addressed to [this folder](https://github.com/gwaxG/robot_ws/tree/main/installation) for more instructions.

## Examples
We invite the reader to check the `examples` folder for example videos and the description of used configuration files and files themselves.  

## Launch

It is possible to launch with (server) and without (standlalone) application interface.  
Important note: one experiment can take from 15 mins to several hours, it depends on your machine and number of experiment time steps.

#### Standalone launching    
In case when you want to launch only the simulation, you should follow the convienent ROS+Gazebo workflow.
You start ROS and launch Gazebo as it presented below, then you launch a learning script. 
Here, the script "stables3_launch.py" is an example, you can modify "stables3_launch.json" to change experiment configurations.
Alternatively, you can provide your own "your_own_template.json" and "your_own_launch.py" for your libraries of choice.  
You can find working examples [here](https://github.com/gwaxG/robot_ws/tree/main/examples) and instructions to currently used fields of the json-based configuration [here](https://github.com/gwaxG/robot_ws/tree/main/backend).

1. `roslaunh backend learning.launch gui:=true &`  
2. `roscd backend/scripts/learning/scripts`  
3. `python stables3_launch.py

#### Server launching    
In case you to start the server application that will communicate with the [GUI application](http://github.com/gwaxG/robot-simu),
you have to execute next commands.

1. `roscd backend`   
2. `./bin/master_app`  
3. [GUI launching](http://github.com/gwaxG/robot-simu)  

Description of configuration fields can be found [here](https://github.com/gwaxG/robot_ws/tree/main/backend) and [here](https://github.com/gwaxG/robot_ws/tree/main/examples).  

If you want to launch a particular package you can find its documentation inside.



## Build 
If you have modified any Go-based node, be sure that they are compiled.
To do so, it is necessary to execute `python build.py` in the `robot_ws` folder.
Launching `python build.py` leads to compilation of all nested Go targets located in `cmd` folders and placing binaries into corresponding `bin` folders.

## Packages
***
### simulation
Gazebo environment flat/multi-floor simulation and robot spawning accordingly to the task.
### control
Real/simulated robot control package.
### perception
Feature extraction from the RGBD image.
### monitor
Task progress and criteria observer.
### gym_training
OpenAI gym environment for tracked robots moving indoor.
### draw
Drawing utilities.
### backend
Automation tools which contain a server application and ROS database node.
### plugins
Gazebo plugins necessery to make contact surface motion model working.
***

