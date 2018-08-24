FROM git.ropod.org:4567/ropod/ropod_common:latest

WORKDIR /fleet-management
ADD . /fleet-management/
RUN pip3 install -r requirements.txt
