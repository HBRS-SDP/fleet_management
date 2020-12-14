# FMS setup

## Overview

The `install.sh` script in this repository will help you to setup a development environment for the Fleet Management System. At the moment it considers that the following repositories are actively in development:
* [fleet-management](https://git.ropod.org/ropod/ccu/fleet-management)
* [ropod_common](https://git.ropod.org/ropod/ropod_common)
* [task-planner](https://github.com/ropod-project/task-planner)
* task allocation:
    * [mrta](https://github.com/ropod-project/mrta)
    * [mrta_stn](https://github.com/ropod-project/mrta_stn)

There are a few other repositories which are needed, but which are considered more stable and therefore not installed in development mode:
* [zyre_base](https://github.com/ropod-project/zyre_base)
* [osm_bridge](https://git.ropod.org/ropod/wm/osm_bridge)

## Manual setup

If you want to replicate what the script above is doing, here is a brief summary:

1. Cloning the fms dependencies using `wstool` from [`fms.rosinstall]`()
2. Installing docker and docker-compose (including post installation steps so you don't need sudo!)
3. Creating the `/opt/ropod/ros/` and `/var/log/ropod` directories, and making you the owner
4. Installing `virtualenv` and `virtualenvwrapper`, and creating the `ropod` virtual environment.
5. Installing the python dependencies in development mode:
    * [ropod_common](https://git.ropod.org/ropod/ropod_common)
    * [task-planner](https://github.com/ropod-project/task-planner): The folder also needs to be moved to `/opt/ropod`
    * task allocation:
        * [mrta](https://github.com/ropod-project/mrta)
        * [mrta_stn](https://github.com/ropod-project/mrta_stn)
6. Creates a catkin workspace for the `ropod_ros_msgs` package and configures it to be compiled with python 3.5 and installed in `/opt/ropod/ros/`
7. Sets up the certificate to be able to use ROPOD's docker registry
7. Pulls the required docker images:
    * [`mongo:4.0-xenial`](https://hub.docker.com/_/mongo)
    * [`docker-overpass-api:amk`](https://git.ropod.org/ropod/wm/docker-overpass-api/container_registry)
    * [`ropod-elevator:latest`](https://git.ropod.org/ropod/infrastructure/ropod-elevator/container_registry)
    * [`ropod-elevator:simulator`](https://git.ropod.org/ropod/infrastructure/ropod-elevator/container_registry)

## To test integration
There are a few other repositories that you will need if you need to test the integration of the FMS with the robots. The very basic list includes:
* [ropod_task_executor](https://git.ropod.org/ropod/ropod_task_executor)
* [ropod_com_mediator](https://git.ropod.org/ropod/communication/ropod_com_mediator)

For setting up a complete development environment with the robot, please follow the instructions found [here](https://github.com/ropod-project/tree-ropod).
