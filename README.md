[![pipeline status](https://git.ropod.org/ropod/ccu/fleet-management/badges/master/pipeline.svg)](https://git.ropod.org/ropod/ccu/fleet-management/commits/master)
[![coverage report](https://git.ropod.org/ropod/ccu/fleet-management/badges/master/coverage.svg)](https://git.ropod.org/ropod/ccu/fleet-management/commits/develop)

# Fleet management System

## Installation

The easiest way is to use the script from the [setup](https://git.ropod.org/ropod/ccu/setup) repository.

## Usage

We use docker to run some required components, among those `overpass` and `mongodb`:

```
docker-compose up -d osm mongo
```

To run the Fleet Management System simply run the following:
```
python3 ccu.py
```

## Tests

### Task request tests

 1. Run the ccu
 2. Launch a zyre robot:

 Go to the folder `task_allocation`

```
python3 robot.py ropod_001
```

3. Run the test
```
python3 task_request_test.py
```

## Using docker

A lot of our tests and components have docker-compose services, for example to run the fms using docker run the following: 

    ```
    docker-compose run fms
    ```

