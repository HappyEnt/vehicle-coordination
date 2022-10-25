FROM arm64v8/ros:humble-perception

LABEL description="Container for development"

# add multiline RUN command that updates locale
#RUN locale && \
#    sudo apt update && sudo apt install locales && \
    # sudo locale-gen en_US en_US.UTF-8 && \
    # sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    # export LANG=en_US.UTF-8 && \
    # locale  # verify settings

# RUN sudo apt update && sudo apt install curl gnupg lsb-release
# RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update && apt-get install -y g++ rsync zip openssh-server make valgrind build-essential cmake libboost-dev libboost-all-dev

# RUN sudo apt install ros-humble-ros-base

RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u 1000 dev

RUN echo 'dev:dev' | chpasswd

RUN service ssh start

EXPOSE 22

CMD ["/usr/sbin/sshd","-D"]
