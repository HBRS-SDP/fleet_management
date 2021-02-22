#!/bin/bash

echo "Install Docker? y/n"
read opt

if [ "$opt" = "y" ]; then
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
fi

cd
echo "Creating directory for workspace"
mkdir -p HBRS/SDP_Workspace/SDP
cd HBRS/SDP_Workspace/SDP

echo "Installing python 3.6"
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update -y
sudo apt install python3.6 -y
echo "Installing venv,pip,pip3,curl and  for python3"
sudo apt-get install python3.6-venv -y
python3.6 -m venv SDP
source SDP/bin/activate

sudo apt install python-pip -y
sudo apt install python3-pip -y
python -m pip install --upgrade "pip < 21.0"
python3.6 -m pip install --upgrade "pip < 21.0"
sudo python3 -m pip install --upgrade "pip < 21.0"
sudo python3.6 -m pip install --upgrade "pip < 21.0"
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install python-catkin-pkg -y
sudo apt install cmake

echo "Cloning fleet management repo"
git clone -b develop https://github.com/HBRS-SDP/fleet_management.git

sudo apt install autoconf -y
pip install -U wstool
python3.6 -m pip install -U wstool

cd
cd HBRS/SDP_Workspace/SDP/fleet_management
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
cd HBRS/SDP_Workspace/SDP
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

su -l ${USER}

#The scripte needs to restart from here
cd
cd HBRS/SDP_Workspace
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
