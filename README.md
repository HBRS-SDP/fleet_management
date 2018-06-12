# Fleet management System

## Dependencies
* libsodium (optional)
* libzmq
* czmq
* Zyre (see [here](https://git.ropod.org/ropod/communication/ropod_com_mediator/blob/master/doc/ropod_dependencies.md) for how to install)
* jsoncpp
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)


## Add the navigation goal constants

```
cd
git clone gitgate@mas.b-it-center.de:ropod/ropod_common.git
sudo mv ropod_common /opt/
```

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
