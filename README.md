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


## Installation

Get the requirements:

```
pip3 install -r requirements.txt
```

Note: This assumes that the PyreBaseCommunicator from [ropod_common](https://git.ropod.org/ropod/ropod_common) has been setup for development. For more information, follow the instructions on the README.

To add the fleet_management to you `PYTHONPATH` simply run:


```
sudo pip3 install -e .
```


## Usage

```
docker-compose up -d osm
```

```
docker-compose up -d mongo
```

```
python3 ccu.py
```

You can run a few of the tests found in the test subfolder.


# Using docker

1. [Install docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
2. [Install docker-compose](https://docs.docker.com/compose/install/)
3. You should be able to log in to ROPOD's docker registry

    ```
    docker login git.ropod.org:4567
    ```

3. Using docker-compose, we will get the necessary docker images, run the containers and existing test services:

    ```
    docker-compose build
    ```
    
4. To run the fms run

    ```
    docker-compose run fms
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
    python3 ccu.py
    ```
