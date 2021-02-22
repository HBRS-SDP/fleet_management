#!/bin/bash

sudo apt-get install software-properties-common -y
sudo apt-get update -y
echo "Installing python 3.6"
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update -y
sudo apt install python3.6 -y
echo "Installing venv,pip,pip3,curl and  for python3"
sudo apt-get install python3.6-venv -y
cd
echo "Creating directory for workspace"
mkdir -p HBRS/SDP_Workspace/SDP
cd HBRS/SDP_Workspace/SDP
python3.6 -m venv SDP
source SDP/bin/activate
sudo apt-get install python
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

echo "Install Python6 and other dependencies: y/n"
#sudo apt remove --purge python3-apt
#sudo apt install python3-apt

sudo apt install python-pip -y
sudo apt install python3-pip
sudo apt install curl
sudo python -m pip install --upgrade "pip < 21.0"
#sudo python3 -m pip install --upgrade "pip < 21.0"
sudo python3.6 -m pip install --upgrade "pip < 21.0"
python -m pip install --upgrade "pip < 21.0"
python3.6 -m pip install --upgrade "pip < 21.0"
sudo apt install git
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-catkin-pkg
sudo python -m pip install -U catkin_pkg
sudo python3 -m pip install -U catkin_pkg
sudo python3.6 -m pip install -U catkin_pkg

echo "Set python6 as default: y/n"

#sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2
#sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 2


cd
cd HBRS/SDP_Workspace/SDP
source SDP/bin/activate
echo "Cloning fleet management repo"
git clone -b develop https://github.com/HBRS-SDP/fleet_management.git

sudo apt install autoconf -y
sudo pip install -U wstool
sudo python3 -m pip install -U wstool
sudo python3.6 -m pip install -U wstool

echo "Install Docker? y/n"

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io -y
echo "Installing docker-compose"
sudo curl -L "https://github.com/docker/compose/releases/download/1.23.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
echo "Running docker post-install steps..."
echo "Adding user to docker group - now you don't need sudo to run docker!"
sudo groupadd docker
sudo usermod -aG docker $USER
echo "Starting docker on startup"
sudo systemctl enable docker

