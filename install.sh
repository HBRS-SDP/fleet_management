#!/bin/bash

# Installing basic ropod libraries
echo "Installing zmq and zyre dependencies"
sudo apt install autoconf python3-pip python-wstool
#mkdir ../dependencies && cd ../dependencies
#bash -c "$(curl -fsSL https://raw.githubusercontent.com/ropod-project/ropod-base-cpp/master/install_deps.sh)"

echo "Installing docker"
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
cd ../setup
wstool init ..
wstool merge -t .. fms.rosinstall
wstool update

sudo mkdir -p /var/log/ropod/fms
sudo mkdir -p /opt/ropod/
sudo chown -R $USER:$USER /opt/ropod/
sudo chown -R $USER:$USER /var/log/ropod 

echo "Installing virtualenv"
pip3 install --user virtualenv virtualenvwrapper
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source $HOME/.local/bin/virtualenvwrapper.sh 
mkvirtualenv ropod
echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3" >> ~/.bashrc
echo "source $HOME/.local/bin/virtualenvwrapper.sh" >> ~/.bashrc

cd ../ropod_common/pyropod
pip3 install -r requirements.txt
pip3 install -e .

echo "Installing fmlib in development mode..."
cd ../fmlib
pip3 install -r requirements.txt
pip3 install -e .

cd ../../
ls
# sudo cp -rf ropod_common /opt/ropod/ # This is not necessary for the FMS

echo "Copying task-planner to /opt/ropod/"
sudo cp -rf task-planner /opt/ropod/
cd task-planner
echo "Installing task-planner in development mode..."
#sudo pip3 install -r requirements.txt
pip3 install -e .

cd ../mrta
echo "Installing mrta in development mode..."
pip3 install -r requirements.txt
pip3 install -e .

cd ../mrta_stn
echo "Installing mrta_stn in development mode..."
pip3 install -r requirements.txt
pip3 install -e .

cd ../fleet-management
echo "Installing fleet-management in development mode..."
pip3 install -r requirements.txt
pip3 install -e .

echo "Setting up the ropod_ros_msgs"
sudo apt install ros-kinetic-core python-catkin-tools
cd ../
mkdir -p ropod_msgs_ws/src
cd ropod_msgs_ws
catkin config
catkin config --extend /opt/ros/kinetic
mkdir -p /opt/ropod/ros/
catkin config --install --install-space /opt/ropod/ros/
catkin config --cmake-args -DPYTHON_VERSION=3.5
cd src
git clone git@git.ropod.org:ropod/communication/ropod_ros_msgs.git
cd ../
catkin build
echo "source /opt/ropod/ros/setup.bash" >> ~/.bashrc

cd ../setup
echo "Getting the required docker images"
sudo mkdir -p /etc/docker/certs.d/git.ropod.org:4567/
sudo cp git.ropod.org.crt /etc/docker/certs.d/git.ropod.org:4567/ca.crt
sudo cp git.ropod.org.crt /usr/local/share/ca-certificates/git.ropod.org.crt
sudo update-ca-certificates
sudo service docker restart
echo "Please login to the ropod registry:"
docker login git.ropod.org:4567
docker pull git.ropod.org:4567/ropod/wm/docker-overpass-api:amk
docker pull git.ropod.org:4567/ropod/infrastructure/ropod-elevator:latest
docker pull git.ropod.org:4567/ropod/infrastructure/ropod-elevator:simulator
docker pull mongo:4.0-xenial
