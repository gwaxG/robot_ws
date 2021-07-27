# Installation

This folder contains the Python environment description and installation bash script.
We are sure that `installation.sh` will work for an empty Ubuntu 20.04 which is also mandatory, 
otherwise we address to check every component from the script and install manually what you lack.  

To install:  
    `chmod +x installation.sh && ./installation.sh`  

Components:  
1. ROS Noetic with ros-control, ros-controllers, controller-manager, geometry2, hector_models, gazebo_ros_pkgs;
2. Gazebo 11 with plugins for the contact surface motion model and robot control;
3. Go latest;
4. Conda and Python libraries (see `environment.yml`);
5. MongoDB.
