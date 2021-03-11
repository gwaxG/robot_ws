#!/usr/bin/bash


echo "Rsync backend"
#  rsync -a . amitriakov@gazebo1.enstb.org:/home/amitriakov/catkin_ws/src/robot_ws/
echo "Backend copied"
echo "Starting build"
ssh amitriakov@gazebo1.enstb.org '~/anaconda3/envs/sb_learning/bin/python /home/amitriakov/catkin_ws/src/robot_ws/build.py'
echo "Build finished"
