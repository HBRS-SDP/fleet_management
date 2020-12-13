#!/bin/bash
sudo apt install curl
echo "Install ROS"


#install ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

sudo apt-get update

##Installing full ros
sudo apt-get install ros-kinetic-desktop-full

apt-cache search ros-kinetic

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo apt install python-rosdep

sudo rosdep init
rosdep update

#configuring environment
printenv | grep ROS
source /opt/ros/kinetic/setup.bash

#creating dummy workspace
mkdir -p ~/catkin_ws_trial/src
cd ~/catkin_ws_trial/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH

echo "Install Python6 and other dependencies: y/n"

echo "Installing python 3.6"
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.6
echo "Installing venv,pip,pip3,curl and  for python3"
sudo apt-get install python3.6-venv
sudo apt install python-pip
sudo apt install python3-pip
sudo apt install curl
sudo python -m pip install --upgrade pip
sudo python3 -m pip install --upgrade pip
sudo python3.6 -m pip install --upgrade pip
sudo apt install git
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-catkin-pkg
sudo python -m pip install -U catkin_pkg
sudo python3 -m pip install -U catkin_pkg
sudo python3.6 -m pip install -U catkin_pkg

echo "Set python6 as default: y/n"

sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.5 1

cd
echo "Creating directory for workspace"
mkdir -p HBRS/SDP_Workspace/SDP
cd HBRS/SDP_Workspace/SDP
echo "Cloning fleet management repo"
git clone -b updateInstallfiles https://github.com/HBRS-SDP/fleet_management.git

sudo apt install autoconf
sudo pip install -U wstool
sudo python3 -m pip install -U wstool
sudo python3.6 -m pip install -U wstool

echo "Install Docker? y/n"

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
echo "Installing docker-compose"
sudo curl -L "https://github.com/docker/compose/releases/download/1.23.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
echo "Running docker post-install steps..."
echo "Adding user to docker group - now you don't need sudo to run docker!"
sudo groupadd docker
sudo usermod -aG docker $USER
echo "Starting docker on startup"
sudo systemctl enable docker



# Installing FMS dependencies
echo "Cloning FMS dependencies"
cp fms.rosinstall ../fms.rosinstall
cd ..
wstool init ..
wstool merge -t .. fms.rosinstall
wstool update

sudo mkdir -p /var/log/ropod/fms
sudo mkdir -p /opt/ropod/
sudo chown -R $USER:$USER /opt/ropod/
sudo chown -R $USER:$USER /var/log/ropod 

cd ../ropod_common/pyropod
pip3 install --user -r requirements.txt
sudo -H pip3 install -e .

cd ../../zyre_base
pip3 install --user -r requirements.txt
sudo pip3 install -e .

echo "Installing fmlib in development mode..."
cd ../fmlib
#sudo pip3 install -e .

echo "Copying task-planner to /opt/ropod/"
sudo cp -rf ../task-planner /opt/ropod/
cd ../task-planner
echo "Installing task-planner in development mode..."
#sudo pip3 install -r requirements.txt
sudo pip3 install -e .
source ./install_LAMA_planner.sh

cd ../mrta
echo "Installing mrta in development mode..."
pip3 install --user -r requirements.txt
pip3 install -e .

cd ../mrta_stn
echo "Installing mrta_stn in development mode..."
pip3 install --user -r requirements.txt
sudo pip3 install -e .

cd ../osm_bridge
sudo pip3 install --user -e .

cd ../fleet-management
echo "Installing fleet-management in development mode..."
sed -i 's|git+https://github.com:/ropod-project/rospy_message_converter.git|git+https://github.com:/ropod-project/rospy_message_converter.git@master|g' requirements.txt
pip3 install --user -r requirements.txt
sudo pip3 install -e .

echo "install catkin tools"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
sudo pip install -U catkin_tools
python -m pip install empy
python3 -m pip install empy
python3.6 -m pip install empy

echo "Setting up the ropod_ros_msgs"
cd ../
mkdir -p ropod_msgs_ws/src
cd ropod_msgs_ws
catkin config
catkin config --extend /opt/ros/kinetic
mkdir -p /opt/ropod/ros/
catkin config --install --install-space /opt/ropod/ros/
catkin config --cmake-args -DPYTHON_VERSION=3.5
cd src
git clone https://github.com/ropod-project/ropod_ros_msgs.git
cd ../
catkin build
echo "source /opt/ropod/ros/setup.bash" >> ~/.bashrc

cd ..
echo "Getting the required docker images"
echo "Please enter system password to log in to docker agian and run script install_native_v2_2"
su -l ${USER}
