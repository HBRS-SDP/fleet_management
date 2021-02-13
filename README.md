![Fleet Management CI](https://github.com/HBRS-SDP/fleet_management/workflows/Fleet%20Management%20CI/badge.svg?branch=develop)

# Fleet management System

## Installation

The easiest way is to use the script from the [setup](https://git.ropod.org/ropod/ccu/setup) repository.

### Native Installation
Note : You will be required to generate a personal token.
To install the Fleet Management System with all dependencies(ROS, Python3.6,Docker, and etc) as a native install (not using virtualenv) run the following commands:
```
cd

cd Downloads

sudo apt install git

sudo apt-get update

sudo apt-get upgrade

git clone -b develop \ 
https://github.com/HBRS-SDP/fleet_management.git

cd fleet_mangement

.\install_native_v2_1.sh

cd HBRS/SDP_Workspace/SDP/fleet_management

.\install_native_v2_2.sh
```
## Usage

We use docker to run some of the required components, among those `overpass` and `mongodb`:

```
docker-compose up -d osm mongo
```

Keep in mind that the default map is now set to the `brsu` overpass image.

To run the Fleet Management System simply run the following:
```
python3 ccu.py
```

## Tests

### Task request tests

 1. Run the ccu as explained above
 2. Launch a zyre robot:  

```
cd fleet_management/proxies/
python3 robot.py ropod_001
```

3. Run the test using `--case <X>` or `--all`
```
python3 task_request_test.py
```

  By default, the `task_request_test` is using the option `--case 4` of the [available test cases](fleet_management/test/fixtures/msgs/task/requests/brsu/test-cases.yaml)

## Using docker

We use docker mostly to test components in GitLab's continuous integration and for deployment purposes, so
a lot of our tests and components have docker-compose services. As an example, to run the fms using docker you can do the following: 

```
docker-compose run fms
```

To see what else is using docker (and how) check the [`.gitlab-ci.yml` file](.gitlab-ci.yml)
