[![pipeline status](https://git.ropod.org/ropod/ccu/fleet-management/badges/master/pipeline.svg)](https://git.ropod.org/ropod/ccu/fleet-management/commits/master)

# Fleet management System

## Components

Task Manager:
- Receives the requests
- Creates the task and instantiates it
- Keeps a schedule of tasks
- Triggers the execution

Task Planner:
- Returns a list of actions

Path planner:
- Returns a list of waypoints and actions, e.g. GO TO A, CALL ELEVATOR FLOOR X

Resource Manager
- Interfaces to building control
- Selects the best robots for a given task
- Maintains a schedule of robots' availabilities

Task monitoring:
- Remote monitoring of the task execution
- Triggers recovery actions and requests replans if needed

Task execution
- Iterates through the list of actions one by one
- Requests the elevator to the resource manager


## Dependencies
* libsodium (optional)
* libzmq
* czmq
* Zyre (see [here](https://git.ropod.org/ropod/communication/ropod_com_mediator/blob/master/doc/ropod_dependencies.md) for how to install)
* jsoncpp
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)
* [mongocxx 3.2.0](https://github.com/mongodb/mongo-cxx-driver)

## Build

```
mkdir build
cd build
cmake ..
make
```

Note: This assumes that [ropod_common](https://git.ropod.org/ropod/ropod_common) can be found in `/opt/ropod/`. For more information, follow the instructions on the README.

## Usage

```
./ccu_cli
```

Select items from the menu to send the different JSON messages specified [here](https://git.ropod.org/ropod/communication/ropod_com_mediator/blob/master/doc/ropod_msgs.md).



# Using docker

1. [Install docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
2. [Install docker-compose](https://docs.docker.com/compose/install/)
3. You should be able to log in to ROPOD's docker registry

    ```
    docker login git.ropod.org:4567
    ```

3. Using docker-compose, we will get the necessary docker images, run the containers and existing test services:

    ```
    docker-compose up
    ```


## Using the fleet-management image to test your local code:
1. Run the fms container:

    ```
      docker-compose run local_test
    ```

2. On a new terminal attach to the `fms-test` container
  ```
  docker attach fms-test
  ```

3. Now you run your tests as usual, e.g. running the ccu:

    ```
    mkdir build && cd build
    cmake ..
    make
    ./ccu
    ```
