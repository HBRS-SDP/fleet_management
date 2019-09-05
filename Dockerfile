FROM git.ropod.org:4567/ropod/docker/ropod-base:fms

RUN git clone https://github.com/ropod-project/task-planner.git /opt/ropod/task-planner
WORKDIR /opt/ropod/task-planner
RUN python3 setup.py install

WORKDIR /opt/ropod/fms/fleet-management
COPY . /opt/ropod/fms/fleet-management/

RUN sh /opt/ros/kinetic/setup.sh \
    && pip3 install -r requirements.txt && pip3 install -e .

WORKDIR fleet_management
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "ccu.py"]
