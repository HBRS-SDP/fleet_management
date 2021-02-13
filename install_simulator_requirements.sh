#!/bin/bash

sudo apt-get install libmongoc-1.0-0
sudo apt-get install libbson-1.0
sudo apt-get install cmake libssl-dev libsasl2-dev
wget https://github.com/mongodb/mongo-c-driver/releases/download/1.16.2/mongo-c-driver-1.17.0.tar.gz
tar xzf mongo-c-driver-1.17.0.tar.gz
cd mongo-c-driver-1.17.0
mkdir cmake-build
cd cmake-build
cmake -DENABLE_AUTOMATIC_INIT_AND_CLEANUP=OFF ..
sudo make install
git clone https://github.com/mongodb/mongo-cxx-driver.git --branch releases/stable --depth 1
cd mongo-cxx-driver/build
sudo cmake ..                                \
    -DCMAKE_BUILD_TYPE=Release          \
    -DBSONCXX_POLY_USE_MNMLSTC=1                      \
    -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make EP_mnmlstc_core
sudo make
sudo make install

sudo apt-get install ros-kinetic-navigation
sudo apt-get install libjsoncpp-dev
sudo apt-get install ros-distro-sbpl
echo 'deb http://download.opensuse.org/repositories/network:/messaging:/zeromq:/release-stable/xUbuntu_16.04/ /' | sudo tee /etc/apt/sources.list.d/network:messaging:zeromq:release-stable.list
curl -fsSL https://download.opensuse.org/repositories/network:messaging:zeromq:release-stable/xUbuntu_16.04/Release.key | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/network_messaging_zeromq_release-stable.gpg > /dev/null
sudo apt update
sudo apt install libzmq3-dev

#installcmq
git clone git://github.com/zeromq/czmq.git
cd czmq
sh autogen.sh
./configure
make all
sudo make install
sudo ldconfig

#Building zyre
git clone --depth 1 -b stable https://github.com/jedisct1/libsodium.git
cd libsodium
./autogen.sh && ./configure && make check
sudo make install
cd ..

git clone git://github.com/zeromq/libzmq.git
cd libzmq
./autogen.sh
# do not specify "--with-libsodium" if you prefer to use internal tweetnacl
# security implementation (recommended for development)
./configure --with-libsodium
make check
sudo make install
sudo ldconfig
cd ..

git clone git://github.com/zeromq/czmq.git
cd czmq
./autogen.sh && ./configure && make check
sudo make install
sudo ldconfig
cd ..

git clone git://github.com/zeromq/zyre.git
cd zyre
./autogen.sh && ./configure && make check
sudo make install
sudo ldconfig
cd ..


sudo apt-get install python-defusedxml


sudo apt install python3-netifaces
