FROM ubuntu:xenial
RUN apt-get update && apt-get install -y apt-utils && apt-get install -y \
curl && apt-get -y install sudo && apt-get install -y lsb-release && apt-get install -y grep

RUN echo "root:root" | chpasswd
RUN sudo apt-get remove docker docker-engine docker.io containerd runc && \
sudo apt-get update && \
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common -y && \
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add - && \
sudo apt-key fingerprint 0EBFCD88 && \
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable" && \
sudo apt-get update -y && \
sudo apt-get install docker-ce docker-ce-cli containerd.io -y && \
sudo curl -L "https://github.com/docker/compose/releases/download/1.28.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose && \
sudo chmod +x /usr/local/bin/docker-compose
#RUN sudo groupadd docker
#RUN sudo usermod -aG docker root
#RUN echo "Starting docker on startup"
#RUN sudo systemctl enable docker

CMD ["echo","Hello"]

CMD /bin/bash
