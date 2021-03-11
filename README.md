# Robot work space
***
## Description
This work space contains all code necessary for jaguar simulation,
control, navigation and learning.

## Launch:
Before starting your work, be sure that Go nodes are compiled.
To do so, it is necessary to execute `go build` in every package 
of every cmd folder (they are not so many btw). The tool like 
catkin_make is coming.

If you want to launch a particular package you can find its documentation inside.
In case of a scenario launch:
* learning: 

    `roslaunh launch_scenario learning.launch`;
* deploy in simulation or reality
  
  `roslaunh launch_scenario deploy.launch`.

## Build:  
This work space uses both Python and Go, therefore a python tool to build Go packages is provided.  
Launch `python build.py` and all nested Go targets located in `cmd` folders will be built and placed into package `bin` folders.

## Packages:
***
### launch_scenario
Launch files for learning and deploy scenarios.
***
### simulation
Gazebo environment flat/multi-floor simulation and robot spawning accordingly to the task.
***
### control
Real/simulated robot control package.
### perception
Feature extraction from the RGBD image.
### monitor
Task progress and criteria observer.
***

## Dependencies  
sshpass & rsync
