#!/bin/bash
cd
cd HBRS/SDP_Workspace
echo -n "Enter your github username: "
read USERNAME
echo -n "Enter github token: "
read CR_PAT
echo $CR_PAT | docker login ghcr.io -u $USERNAME --password-stdin

docker pull ghcr.io/hbrs-sdp/ropod-elevator:simulator
docker pull ghcr.io/hbrs-sdp/ropod-elevator:latest
docker pull mongo:4.0-xenial

git clone https://github.com/ropod-project/docker-overpass-api.git
docker build -t hbrs-sdp/docker-overpass-api:latest docker-overpass-api/

git clone https://github.com/ropod-project/ropod_com_mediator.git
cd ropod_com_mediator
sed -i 's+git.ropod.org:4567/ropod/ropod_common:latest+ropod/ropod_common:latest+g' Dockerfile
docker build -t ropod/ropod_com_mediator:latest .

cd ../SDP/fleet_management
sed -i 's+FROM ropod-base:fms+FROM ropod/ropod-base:fms+g' Dockerfile
sed -i 's+image: hbrs-sdp/ropod_com_mediator:latest+image: ropod/ropod_com_mediator:latest+' docker-compose.yml

python3 -m pip install rospkg
python -m pip install rospkg
python3.6 -m pip install rospkg


