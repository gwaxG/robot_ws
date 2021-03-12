#!/usr/bin/bash


echo "Starting build"
python build.py
echo "Build finished"
echo "Synchronization started"
rsync -a . amitriakov@gazebo1.enstb.org:/home/amitriakov/catkin_ws/src/robot_ws/
echo "Synchronization finished"
echo "Starting backend master app"
ssh amitriakov@gazebo1.enstb.org '/home/amitriakov/catkin_ws/src/robot_ws/backend/bin/master_app' &
disown %1
echo "END"
