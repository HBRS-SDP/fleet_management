FROM ubuntu:xenial
RUN apt-get update && apt-get install -y apt-utils && apt-get install -y \
curl && apt-get -y install sudo && apt-get install -y lsb-release && apt-get install -y grep

RUN echo "root:root" | chpasswd
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

RUN sudo apt-get update
RUN sudo apt-get install ros-kinetic-desktop-full -y
RUN apt-cache search ros-kinetic
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"
RUN sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
RUN sudo apt-get install software-properties-common -y
RUN sudo apt-get update
RUN sudo apt-get install python-rosdep
RUN sudo rosdep init
RUN rosdep update
#RUN printenv
#RUN printenv | grep ROS
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"
RUN sudo add-apt-repository ppa:deadsnakes/ppa
RUN sudo apt update
RUN sudo apt install python3.6 -y
RUN sudo apt-get install python -y
RUN sudo apt install python-pip -y
RUN sudo apt install python3-pip -y
RUN sudo apt install curl
RUN sudo python -m pip install --upgrade "pip < 21.0"
RUN sudo python3.6 -m pip install --upgrade "pip < 21.0"
RUN sudo apt install git
RUN sudo apt-get install python-catkin-pkg
RUN sudo python -m pip install -U catkin_pkg
RUN sudo python3.6 -m pip install -U catkin_pkg

RUN sudo apt install autoconf -y
RUN sudo pip install -U wstool
RUN sudo python3 -m pip install -U wstool
RUN sudo python3.6 -m pip install -U wstool
RUN apt-get install apt-transport-https ca-certificates -y
RUN sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 2
RUN echo "Install Docker? y/n"

RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
RUN sudo apt-key fingerprint 0EBFCD88
RUN sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
RUN sudo apt update -y
RUN sudo apt install docker-ce docker-ce-cli containerd.io -y
RUN echo "Installing docker-compose"
RUN sudo curl -L "https://github.com/docker/compose/releases/download/1.23.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
RUN sudo chmod +x /usr/local/bin/docker-compose
RUN echo "Running docker post-install steps..."
RUN echo "Adding user to docker group - now you don't need RUN sudo to run docker!"
#RUN sudo groupadd docker
#RUN sudo usermod -aG docker $USER
#RUN echo "Starting docker on startup"
#RUN sudo systemctl enable docker

RUN sudo apt-get install wget && apt-get update && apt-get install ca-certificates
RUN echo "install catkin tools"
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install python-catkin-tools -y
RUN sudo pip2 install -U catkin_tools
RUN python -m pip install empy
RUN python3.6 -m pip install empy
RUN sudo python3 -m pip install -U catkin_pkg
RUN echo "Setting up ros workspace"

RUN cd
RUN sudo mkdir -p /opt/ropod/ros_workspace/src
RUN cd /opt/ropod/ros_workspace/ && sudo catkin config --extend /opt/ros/kinetic && catkin config --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && sudo catkin build && echo "source /opt/ropod/ros_workspace/devel/setup.bash" >> ~/.bashrc

RUN cd && echo "Creating directory for workspace" && mkdir -p HBRS/SDP_Workspace/SDP && cd HBRS/SDP_Workspace/SDP && git clone -b develop https://github.com/HBRS-SDP/fleet_management.git && cd fleet_management/ && echo "Cloning FMS dependencies" && cp fms.rosinstall ../fms.rosinstall && cd .. && wstool init .. && wstool merge -t .. fms.rosinstall && wstool update

RUN sudo mkdir -p /var/log/ropod/fms
RUN sudo mkdir -p /opt/ropod/
RUN sudo chown -R $USER:$USER /opt/ropod/
RUN sudo chown -R $USER:$USER /var/log/ropod

RUN cd && cd HBRS/SDP_Workspace/SDP && cd ../ropod_common/pyropod && pip3 install --user -r requirements.txt && sudo -H pip3 install -e . && cd ../../zyre_base && pip3 install --user -r requirements.txt && sudo pip3 install -e . && echo "Installing fmlib in development mode..." && cd ../fmlib && echo "Copying task-planner to /opt/ropod/" && sudo cp -rf ../task-planner /opt/ropod/ && cd ../task-planner && echo "Installing task-planner in development mode..." && sudo pip3 install -e . && sudo ./install_LAMA_planner.sh

RUN cd && cd HBRS/SDP_Workspace/SDP && cd ../mrta && echo "Installing mrta in development mode..." && pip3 install --user -r requirements.txt && pip3 install -e . && cd ../mrta_stn && echo "Installing mrta_stn in development mode..." && pip3 install --user -r requirements.txt && sudo pip3 install -e . && cd ../osm_bridge && sudo pip3 install --user -e . && cd ../fleet-management && echo "Installing fleet-management in development mode..." && sed -i 's|git+https://github.com:/ropod-project/rospy_message_converter.git|git+https://github.com:/ropod-project/rospy_message_converter.git@master|g' requirements.txt && pip3 install --user -r requirements.txt && sudo pip3 install -e .

RUN sudo usermod -aG docker root
RUN echo "Starting docker on startup"
RUN sudo systemctl enable docker

RUN su -l root

CMD ["echo","Hello"]

CMD /bin/bash
