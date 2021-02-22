#!/bin/bash
sudo apt install curl -y
echo "Install ROS"


#install ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

sudo apt-get update

##Installing full ros
sudo apt-get install ros-kinetic-desktop-full -y

apt-cache search ros-kinetic

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y


sudo apt install python-rosdep

sudo rosdep init
rosdep update

#configuring environment
printenv | grep ROS
source /opt/ros/kinetic/setup.bash

echo "install catkin tools"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools -y
sudo pip install -U catkin_tools
sudo apt install python-pip -y
sudo apt install python3-pip -y
python -m pip install empy
python3.6 -m pip install empy
sudo apt-get install python-catkin-pkg
sudo python -m pip install -U catkin_pkg
sudo python3 -m pip install -U catkin_pkg
sudo python3.6 -m pip install -U catkin_pkg


echo "Setting up ros workspace"
cd
sudo mkdir -p /opt/ropod/ros_workspace/src
cd /opt/ropod/ros_workspace/
sudo catkin config --extend /opt/ros/kinetic
sudo catkin config --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
#cd /opt/ropod/ros_workspace
catkin build
echo "source /opt/ropod/ros_workspace/devel/setup.bash" >> ~/.bashrc
