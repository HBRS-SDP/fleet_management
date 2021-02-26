![Fleet Management CI](https://github.com/HBRS-SDP/fleet_management/workflows/Fleet%20Management%20CI/badge.svg?branch=develop)

# Fleet management System

### FMS and Simulator Installation
Note : 
1) You will be required to generate a personal token.
2) If you choose to install docker using the following script, then you will have to re-run the install script and the next time do not select to install docker again.

To install all the requirements of the fleet management system, you can run the following steps:
```
cd

cd Downloads

sudo apt install git

git clone -b develop \ 
https://github.com/HBRS-SDP/fleet_management.git

cd fleet_mangement

./install_fms

./install_simulator
```
## Usage

We use docker to run some of the required components, such as `mongodb`:

```
docker-compose up -d mongo
```

The map for the toplolgical map is the `brus-full` map by default.

To run the Fleet Management System simply run the following:
```
python3 ccu.py --config=topology
```
To run the Simulator simply run the following:
```
roslaunch ropod_gazebo multi_robot.launch
```

## Tests

### Task request tests

 1. Run the ccu as explained above
 2. Launch a zyre robot:  

```
cd fleet_management/proxies/
python3 robot.py ropod_001 --config=topology
python3 robot.py ropod_002 --config=topology
```

3. Run the test using `--case <X>` or `--all`
```
python3 task_request_test.py --config=topology
```

  By default, the `task_request_test` is using the option `--case 4` of the [available test cases](fleet_management/test/fixtures/msgs/task/requests/brsu/topology-test-cases.yaml)

