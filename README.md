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

## Build

```
mkdir build
cd build
cmake ..
make
```

## Usage

```
./ccu_cli
```

Select items from the menu to send the different JSON messages specified [here](https://git.ropod.org/ropod/communication/ropod_com_mediator/blob/master/doc/ropod_msgs.md).
