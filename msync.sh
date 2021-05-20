#!/usr/bin/bash

echo "Synchronization of server models"
rsync -a amitriakov@gazebo1.enstb.org:/home/amitriakov/catkin_ws/src/robot_ws/data/models/server/ ./data/models/server
echo "Synchronization of server boards"
rsync -a amitriakov@gazebo1.enstb.org:/home/amitriakov/catkin_ws/src/robot_ws/data/boards/server/ ./data/boards/server
echo "Synchronization finished"
