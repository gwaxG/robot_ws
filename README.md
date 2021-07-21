# Robot work space (in progress)
***
## Description
This work space contains all code necessary for jaguar simulation,
control, percpetion and learning.

## Build:  
Before starting your work, be sure that Go nodes are compiled.
To do so, it is necessary to execute `python build.py` in the `robot_ws` folder.
Launch `python build.py` and all nested Go targets located in `cmd` folders will be built and placed into package `bin` folders.

## Launch:
If you want to launch a particular package you can find its documentation inside.

Standalone simulation launching.  
"stables3_launch.py" is an example, you can modify "stables3_launch.json" to change experiment configurations.
Alternatively, you can provide your own "your_own_template.json" and "your_own_launch.py" for your libraries of choice.  
Example launching:    
    `roslaunh backend learning.launch &`  
    `roscd backend/scripts/learning/scripts`  
    `python stables3_launch.py`  

Server launching:    
    `roscd backend`   
    `./bin/master_app`  
    [GUI launching](http://github.com/gwaxG/robot-simu)
    

## Packages:
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

## Requirements and installation
The user is addressed to `robot_ws/instalation folder` for more instructions.
