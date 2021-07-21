#!/bin/bash

# ROS + Gazebo with plugins installations

sudo apt -y update && sudo apt -y upgrade

sudo apt -y install git wget

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

# Install plugins and gazebo
cd ~/catkin_ws/src
git clone https://github.com/gwaxG/robot_ws.git

cd ~/catkin_ws/src
git clone https://github.com/gwaxG/robot_ws.git

# Install Gazebo from source with copied custom plugins
git clone https://github.com/osrf/gazebo /tmp/gazebo && \
cp -r ~/catkin_ws/src/robot_ws/plugins/gazebo_plugins/* /tmp/gazebo/plugins && \
cd /tmp/gazebo && \
source /opt/ros/noetic/setup.bash && \
mkdir build && \
cd build && \
cmake ../ && \
make -j4 && \
sudo make install   

# Copy flipper control plugin
cd  ~/catkin_ws/src/robot_ws/plugins/flipper_control
mkdir build
cd build
cmake ..
make -j4
sudo cp libjaguar_plugin.so /usr/lib/

# Modification of env. variables, default path is /usr/local
echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc 
echo "export PATH=/usr/local/bin:$PATH" >> ~/.bashrc
echo "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
source ~/.bashrc

# Install ROS packages
sudo apt-get install -y ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ros-noetic-ros-control ros-noetic-controller-manager ros-noetic-joint-state-controller

cd ~/catkin_ws/src && \
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b noetic-devel && \
    git clone https://github.com/ros/geometry2.git -b 0.7.5 && \
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_models.git && \
    source /opt/ros/noetic/setup.bash && \
    cd ~/catkin_ws && \
    /bin/bash -c "catkin_make"
    
# Python installations
cd ~/ && curl https://repo.anaconda.com/archive/Anaconda3-2020.07-Linux-x86_64.sh --output conda.sh &&\
    chmod +x ~/conda.sh &&\
    ~/conda.sh -b
echo "export PATH=~/anaconda3/bin:$PATH" >> ~/.bashrc
source ~/.bashrc
conda init bash
source ~/.bashrc
conda env create -f ~/catkin_ws/src/robot_ws/installation/environment.yml
  
echo "conda activate sb_learning" >> ~/.bashrc && source ~/.bashrc

cd ~/catkin_ws/src/robot_ws/gym-training && pip install -e .
  
# Go installation
cd ~/Downloads
wget https://golang.org/dl/go1.16.6.linux-amd64.tar.gz
rm -rf /usr/local/go 
sudo tar -C /usr/local -xzf go1.16.6.linux-amd64.tar.gz
echo "export PATH=$PATH:/usr/local/go/bin" >> ~/.bashrc
source ~/.bashrc

# MongoDB installation
sudo apt-get install -y gnupg
wget -qO - https://www.mongodb.org/static/pgp/server-5.0.asc | sudo apt-key add -
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/5.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-5.0.list
sudo apt-get update
sudo apt-get install -y mongodb-org

echo "running=\`pgrep mongod\`
if  [[ !  -z  \$running  ]]
then
  echo \"Mongo is already running\" \$running
else
  sudo systemctl start mongod
  echo \"Mongo is started\"
fi
">> ~/.bashrc

# Project initialization
roscd control/..
python build.py


