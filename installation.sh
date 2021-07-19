#!/bin/bash

sudo apt-get -y update && sudo apt-get -y upgrade

sudo apt -y install git

# install ros noetic desktop full
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
  sudo apt update && \
  sudo apt install -y ros-noetic-desktop-full && \
  source /opt/ros/noetic/setup.bash && \
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
  source ~/.bashrc

# uninstall gazebo if it was installed
sudo apt-get -y remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'

# Ignition CMake
sudo apt-get -y install build-essential cmake pkg-config && \
	git clone https://github.com/ignitionrobotics/ign-cmake /tmp/ign-cmake && \
	cd /tmp/ign-cmake && \
	git checkout ign-cmake2 && \
	mkdir build && \
	cd build && \
	cmake ../ && \
	make -j4 && \
	sudo make install

# Ignition Math
sudo apt-get -y install build-essential cmake git python && \
	git clone https://github.com/ignitionrobotics/ign-math /tmp/ign-math && \
	cd /tmp/ign-math && \
	git checkout ign-math6 && \
	mkdir build && \
	cd build && \
	cmake ../ && \
	make -j4 && \
	sudo make install

# Ignition Common
sudo apt-get -y install build-essential \
         cmake \
         libfreeimage-dev \
         libtinyxml2-dev \
         uuid-dev \
         libgts-dev \
         libavdevice-dev \
         libavformat-dev \
         libavcodec-dev \
         libswscale-dev \
         libavutil-dev \
         libprotoc-dev \
         libprotobuf-dev && \
	git clone https://github.com/ignitionrobotics/ign-common /tmp/ign-common && \
	cd /tmp/ign-common && \
	git checkout ign-common3 && \
	mkdir build && \
	cd build && \
	cmake ../ && \
	make -j4 && \
	sudo make install

# SDFormat
sudo apt-get -y install build-essential \
                     cmake \
                     git \
                     python \
                     libboost-system-dev \
                     libtinyxml-dev \
                     libxml2-utils \
                     ruby-dev \
                     ruby && \
	git clone https://github.com/osrf/sdformat /tmp/sdformat && \
	cd /tmp/sdformat && \
	git checkout sdf9 && \
	mkdir build && \
	cd build && \
	cmake ../ && \
	make -j4 && \
	sudo make install

# Ignition Messages
sudo apt-get -y install build-essential \
                     cmake \
                     git \
                     libprotoc-dev \
                     libprotobuf-dev \
                     protobuf-compiler && \
	git clone https://github.com/ignitionrobotics/ign-msgs /tmp/ign-msgs && \
	cd /tmp/ign-msgs && \
	git checkout ign-msgs5 && \
	mkdir build && \
	cd build && \
	cmake ../ && \
	make -j4 && \
	sudo make install

# Ignition Fuel Tools
sudo apt-get -y install build-essential \
                     cmake \
         libzip-dev \
         libjsoncpp-dev \
         libcurl4-openssl-dev \
         libyaml-dev && \
	git clone https://github.com/ignitionrobotics/ign-fuel-tools /tmp/ign-fuel-tools && \
	cd /tmp/ign-fuel-tools && \
	git checkout ign-fuel-tools4 && \
	mkdir build && \
	cd build && \
	cmake ../ && \
	make -j4 && \
	sudo make install


# wget
# sudo apt -y install wget && sudo apt -y install vim

# Ignition Transport
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt-get -y update && \
    sudo apt-get -y install libignition-transport8-dev
    
    
# Init workspace
mkdir ~/catkin_ws
mkdir ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd ~/catkin_ws/src
git clone https://github.com/AndreiMitriakov/jaguar_ws.git

# Install Gazebo from source with copied custom plugins
git clone https://github.com/osrf/gazebo /tmp/gazebo
cp -r ~/catkin_ws/src/robot_ws/plugins/gazebo_plugins/* /tmp/gazebo/plugins 

cd /tmp/gazebo
source /opt/ros/noetic/setup.bash
mkdir build
cd build
cmake ../
make -j4 
sudo make install    

# Copy flipper control plugin
cd  ~/catkin_ws/src/jaguar_ws/docker/shared/flipper_control
mkdir build
cd build
cmake ..
make -j4
sudo cp libjaguar_plugin.so /usr/lib/

# cd ~/catkin_ws/src
# sudo apt-get update && sudo apt-get install -y ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ros-noetic-ros-control
# git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel
# git clone https://github.com/ros/geometry2.git -b 0.7.5
# source /opt/ros/noetic/setup.bash
# cd ~/catkin_ws
# /bin/bash -c "catkin_make"


# cd ~/catkin_ws/src
# git clone https://github.com/tu-darmstadt-ros-pkg/hector_models.git &&\
