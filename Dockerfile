FROM blumenthal/ropod-base-cpp:latest

RUN apt-get -y update && apt-get install -y \
    vim \
    git \
    cmake \
    build-essential \
    automake \
    libtool \
    libtool-bin \
    pkg-config \
    wget \
    curl \
    unzip \
    libssl-dev \
    libsasl2-dev

RUN cd /opt
COPY install_deps.sh /opt
RUN cd /opt; chmod 755 install_deps.sh
RUN cd /opt; /bin/bash -c "source ~/.bashrc"; /bin/bash -c "./install_deps.sh --workspace-path=/opt --install-path=/usr/local --no-sudo -j=2"


