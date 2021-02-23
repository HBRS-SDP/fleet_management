#!/bin/bash

echo "Install Docker? y/n"
read opt_docker

if [ "$opt_docker" = "y" ]; then
	sudo apt-get remove docker docker-engine docker.io containerd runc

	sudo apt-get update
	sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common -y

	curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

	sudo apt-key fingerprint 0EBFCD88

	sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"

	sudo apt-get update -y

	sudo apt-get install docker-ce docker-ce-cli containerd.io -y
	sudo curl -L "https://github.com/docker/compose/releases/download/1.23.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
	sudo chmod +x /usr/local/bin/docker-compose

	sudo groupadd docker
	sudo usermod -aG docker $USER
	echo "Starting docker on startup"
	sudo systemctl enable docker
	su -l ${USER}
fi

cd
echo "Creating directory for workspace"
sudo mkdir -p /opt/ropod/fms_Workspace/FMS
cd /opt/ropod/fms_Workspace/FMS

echo "Installing python 3.6? y/n"
read opt_python
if [ "$opt_python" = "y" ]; then
	sudo add-apt-repository ppa:deadsnakes/ppa
	sudo apt update -y
	sudo apt install python3.6 -y
	echo "Installing venv,pip,pip3,curl and  for python3"
	sudo apt-get install python3.6-venv -y
fi
#python3.6 -m venv SDP
#source SDP/bin/activate

sudo apt install python-pip -y
sudo apt install python3-pip -y
python -m pip install --upgrade "pip < 21.0"
python3.6 -m pip install --upgrade "pip < 21.0"
sudo python3 -m pip install --upgrade "pip < 21.0"
sudo python3.6 -m pip install --upgrade "pip < 21.0"
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install python-catkin-pkg -y
sudo apt install cmake -y

echo "Cloning fleet management repo"
git clone -b develop https://github.com/HBRS-SDP/fleet_management.git

sudo apt install autoconf -y
pip install -U wstool
python3.6 -m pip install -U wstool

cd
cd /opt/ropod/fms_Workspace/FMS/fleet_management
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
pip3 install -r requirements.txt
sudo -H pip3 install -e .

cd ../../zyre_base
pip3 install -r requirements.txt
sudo -H pip3 install -e .

echo "Installing fmlib in development mode..."
cd ../fmlib
#sudo pip3 install -e .

echo "Copying task-planner to /opt/ropod/"
sudo cp -rf ../task-planner /opt/ropod/
cd ../task-planner
echo "Installing task-planner in development mode..."
#sudo pip3 install -r requirements.txt
sudo -H pip3 install -e .
sudo source ./install_LAMA_planner.sh

cd 
cd /opt/ropod/fms_Workspace/FMS
cd ../mrta
echo "Installing mrta in development mode..."
pip3 install -r requirements.txt
sudo -H pip3 install -e .

cd ../mrta_stn
echo "Installing mrta_stn in development mode..."
pip3 install -r requirements.txt
sudo -H pip3 install -e .

cd ../osm_bridge
sudo -H pip3 install -e .

cd ../fleet-management
echo "Installing fleet-management in development mode..."
sed -i 's|git+https://github.com:/ropod-project/rospy_message_converter.git|git+https://github.com:/ropod-project/rospy_message_converter.git@master|g' requirements.txt
pip3 install -r requirements.txt
sudo -H pip3 install -e .



#The scripte needs to restart from here
cd
cd /opt/ropod/fms_Workspace
echo -n "Enter your github username: "
read USERNAME
echo -n "Enter github token: "
read CR_PAT
echo $CR_PAT | docker login ghcr.io -u $USERNAME --password-stdin

docker pull ghcr.io/hbrs-sdp/ropod-elevator:simulator
docker pull ghcr.io/hbrs-sdp/ropod-elevator:latest
docker pull mongo:4.0-xenial

git clone https://github.com/ropod-project/docker-overpass-api.git
docker build -t hbrs-sdp/docker-overpass-api:latest docker-overpass-api/

git clone https://github.com/ropod-project/ropod_com_mediator.git
cd ropod_com_mediator
sed -i 's+git.ropod.org:4567/ropod/ropod_common:latest+ropod/ropod_common:latest+g' Dockerfile
docker build -t ropod/ropod_com_mediator:latest .

cd ../SDP/fleet_management
sed -i 's+FROM ropod-base:fms+FROM ropod/ropod-base:fms+g' Dockerfile
sed -i 's+image: hbrs-sdp/ropod_com_mediator:latest+image: ropod/ropod_com_mediator:latest+' docker-compose.yml


sudo apt install curl -y

echo "Install ROS kinetic ? y/n:"
read opt_ros
if [ "$opt_ros" = "y" ]; then

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
fi

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
