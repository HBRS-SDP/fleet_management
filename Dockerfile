FROM git.ropod.org:4567/ropod/ropod_common:latest

RUN git clone https://github.com/ropod-project/task-planner.git /opt/ropod/task-planner
WORKDIR /opt/ropod/task-planner
RUN python3 setup.py install

WORKDIR /opt/ropod/fms/fleet-management
COPY . /opt/ropod/fms/fleet-management/
RUN pip3 install -r requirements.txt && pip3 install -e .
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt update -y && apt upgrade -y && apt install -y ros-kinetic-rospy-message-converter

WORKDIR fleet_management

CMD ["python3", "ccu.py"]
