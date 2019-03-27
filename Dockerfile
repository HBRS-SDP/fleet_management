FROM git.ropod.org:4567/ropod/ropod_common:latest

RUN git clone https://github.com/ropod-project/task-planner.git /opt/ropod/task-planner
WORKDIR /opt/ropod/task-planner
RUN python3 setup.py install

WORKDIR /opt/ropod/fms/fleet-management
COPY . /opt/ropod/fms/fleet-management/
RUN pip3 install -r requirements.txt && pip3 install -e .

WORKDIR fleet_management

CMD ["python3", "ccu.py"]
