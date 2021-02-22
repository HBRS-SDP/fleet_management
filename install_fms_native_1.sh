#!/bin/bash

# Installing FMS dependencies
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

cd 
cd HBRS/SDP_Workspace/SDP
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
python3.6 -m pip install empy

echo "Setting up ros workspace"
cd
mkdir -p /opt/ropod/ros_workspace/src
cd /opt/ropod/ros_workspace/
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
cd /opt/ropod/ros_workspace
catkin build
echo "source /opt/ropod/ros_workspace/devel/setup.bash" >> ~/.bashrc


cd ..
echo "Getting the required docker images"
echo "Please enter system password to log in to docker agian and run script install_native_v2_2"
su -l ${USER}
