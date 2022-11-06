FROM arm64v8/ros:humble-perception
SHELL ["/bin/bash", "-c"]

ARG HOST_SHARED_DIRECTORY
ARG CONTAINER_SHARED_DIRECTORY
ARG ROS_WORKSPACE

# Environment variable for shared folder information
ENV WEBOTS_SHARED_FOLDER=$HOST_SHARED_DIRECTORY:$CONTAINER_SHARED_DIRECTORY

# Create shared directory and ros2 workspace
RUN mkdir -p $CONTAINER_SHARED_DIRECTORY

# Install ROS2 Humble desktop
RUN sudo apt update && sudo apt install curl gnupg lsb-release -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sudo apt update
RUN sudo apt upgrade -y
RUN sudo apt install ros-humble-desktop -y

# Clone webots_ros2 sources and configure for arm64 architecture
RUN source /opt/ros/humble/setup.bash
RUN git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git $ROS_WORKSPACE/src/webots_ros2
RUN cd $ROS_WORKSPACE/src/webots_ros2/webots_ros2_driver/webots && git checkout arm64 && cd $ROS_WORKSPACE

# Install ros dependencies
RUN sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions openssh-server -y
RUN sudo rosdep update
RUN rosdep install --from-paths $ROS_WORKSPACE/src --ignore-src --rosdistro humble -y

# Install Orcar dependencies
RUN sudo apt install build-essential libgsl-dev libboost-all-dev protobuf-compiler -y
RUN sudo pip3 install rpi-hardware-pwm

# Build packages
RUN source /opt/ros/humble/setup.bash && cd $ROS_WORKSPACE && colcon build