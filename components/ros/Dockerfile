FROM osrf/ros:humble-desktop-full

LABEL description="Container for development"

# install build dependencies
RUN apt-get update && apt-get install -y g++ rsync zip openssh-server make valgrind build-essential cmake libboost-dev libboost-all-dev

RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u 1000 dev

RUN echo 'dev:dev' | chpasswd

RUN service ssh start

EXPOSE 22

CMD ["/usr/sbin/sshd","-D"]
