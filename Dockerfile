FROM git.ropod.org:4567/ropod/ropod_common:latest

WORKDIR /temp
COPY install_deps.sh /temp
RUN chmod +x install_deps.sh \
    && ./install_deps.sh --workspace-path=/temp --install-path=/usr/local --no-sudo -j=2

WORKDIR /fleet-management
ADD . /fleet-management/
RUN rm -rf build & mkdir build && cd build && cmake .. \
    && make
