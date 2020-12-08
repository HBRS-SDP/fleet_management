FROM hbrs-sdp/ropod-base:fms

WORKDIR /opt/ropod/fms/fleet-management
COPY . /opt/ropod/fms/fleet-management/

RUN sh /opt/ros/kinetic/setup.sh \
    && pip3 install -r requirements.txt && pip3 install -e .

WORKDIR fleet_management
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "ccu.py"]
