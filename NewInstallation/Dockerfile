FROM ubuntu:xenial
RUN apt-get update && apt-get install -y apt-utils && apt-get install -y \
curl && apt-get -y install sudo && apt-get install -y lsb-release && apt-get install -y grep

RUN echo "root:root" | chpasswd
RUN sudo apt-get install software-properties-common -y
RUN sudo apt-get update

RUN apt-get install apt-transport-https ca-certificates -y

RUN sudo add-apt-repository ppa:deadsnakes/ppa
RUN sudo apt update
RUN sudo apt install python3.6 -y
RUN echo "Install Docker? y/n"

RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
RUN sudo apt-key fingerprint 0EBFCD88
RUN sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
RUN sudo apt update 
RUN sudo apt install docker-ce docker-ce-cli containerd.io -y
RUN echo "Installing docker-compose"
RUN sudo curl -L "https://github.com/docker/compose/releases/download/1.23.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
RUN sudo chmod +x /usr/local/bin/docker-compose
RUN echo "Running docker post-install steps..."
RUN echo "Adding user to docker group - now you don't need RUN sudo to run docker!"


CMD ["echo","Hello"]

CMD /bin/bash
