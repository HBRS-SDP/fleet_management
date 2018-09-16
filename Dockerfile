FROM git.ropod.org:4567/ropod/ropod_common:latest

WORKDIR /opt/ropod/fleet-management
COPY . /opt/ropod/fleet-management/
RUN pip3 install -r requirements.txt && pip3 install -e .

WORKDIR fleet_management

CMD ["python3", "ccu.py"]
