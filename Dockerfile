FROM git.ropod.org:4567/ropod/docker/ropod-base:kinetic-msgs-python3 AS ropod_msgs


FROM git.ropod.org:4567/ropod/ropod_common:latest

WORKDIR /
COPY --from=ropod_msgs /opt/ropod/ros /opt/ropod/ros
COPY ros_entrypoint.sh /

RUN git clone https://github.com/ropod-project/task-planner.git /opt/ropod/task-planner
WORKDIR /opt/ropod/task-planner
RUN python3 setup.py install

WORKDIR /opt/ropod/fms/fleet-management
COPY . /opt/ropod/fms/fleet-management/

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get -y update && apt-get install -y \
    ros-kinetic-ros-core \
    && sh /opt/ros/kinetic/setup.sh \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip
RUN sh /opt/ros/kinetic/setup.sh \
    && pip3 install -r requirements.txt && pip3 install -e .

#RUN apt install -y python3-catkin-pkg \
#    python3-rospkg \



WORKDIR fleet_management
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "ccu.py"]
