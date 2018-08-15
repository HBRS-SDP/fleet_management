FROM git.ropod.org:4567/ropod/ccu/fleet-management:base

WORKDIR /fleet-management
ADD . /fleet-management/
RUN mkdir build && cd build && cmake .. \
    && make
