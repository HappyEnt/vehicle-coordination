=============
Orcar on ROS2
=============

Here provided is a skeleton for running Orcar on `ROS2`_. Robot Operating System, short ROS, is a
middleware that provides among other things primitives for communication between components that
constitute a robot, like sensors, navigation algorithms, ...  Currently implemented is localization
using a variety of particle filter algorithms and a simulation with a virtual model of our Orcar.

Installation
------------

Raspberry Pi
^^^^^^^^^^^^
..  code-block::

    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-humble-ros-base

    sudo apt install python3-colcon-common-extensions

- Enable pwm0 and pwm1:
  In ``/boot/firmware/config.txt`` add ``dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2``

- Add udev rule ``/etc/udev/rules.d/99-com.rules`` so user can access pwm device file:

..  code-block::

    SUBSYSTEM=="pwm*", PROGRAM="/bin/sh -c '\
        chown -R root:gpio /sys/class/pwm && chmod -R 770 /sys/class/pwm;\
        chown -R root:gpio /sys/devices/platform/soc/*.pwm/pwm/pwmchip* && chmod -R 770 /sys/devices/platform/soc/*.pwm/pwm/pwmchip*\
    '"

MacOS Native Installation
^^^^^^^^^^^^^^^^^^^^^^^^^

Using `Robostack`_ a native installation on ROS2 is also possible on
MacOS. Keep in mind that very is a multitude of packages that are not yet running on
Macos/Arm. Simply follow the `installation guide`_ here to install ROS2 using Robostack

A lot of packages, including webots_ros2, are not yet compatible with ROS2 on macos.  You can still
use rviz and other tools on macos using robostack, but it is advisable to run your ros stack inside
a docker container running Ubuntu. Also keep in mind that the discovery mechanism for the default
DDS mechanism uses multicast messages that might not work with container images running in Docker.
Dependencies
^^^^^^^^^^^^
Before trying to build the project make sure that you have a working ROS2 installation.  Currently
we support the humble release of ROS2.  See the official ROS2 `installation documentation`_ for instructions.

ROS2 works best on Ubuntu, for installation on MacOS it is therefore recommended to either use a Docker
container or a Virtual Machine.

Before we can build the project, we will need to install some further ROS packages:

..  code-block::

    sudo apt install ros-humble-webots-ros2 ros-humble-webots-ros2-driver

Furthermore, besides ROS2 the following dependencies have to be installed:

- ``GNU Scientific Library``
- ``Boost``
- ``Protobuf``

Installation on Ubuntu:

..  code-block::

    sudo apt install build-essential python3-pip libgsl-dev libboost-all-dev protobuf-compiler
    sudo pip3 install rpi-hardware-pwm protobuf==3.19.6 # TODO this should normally not be needed. Ubuntu dependency is not found for some reason.

Building
^^^^^^^^
In order to build the project, simply run colcon build from the workspace folder.  Afterwards you
will have to source ``install/local_setup.bash`` so the newly build packages can be found by
ROS. Make sure you executed ``source /opt/ros/humble/setup.bash`` manually or by placing it into
your ``.profile``, before executing the following commands

..  code-block::

    cd ~/vehicle-coordination/components/ros
    colcon build
    source install/local_setup.bash

Look into the `colcon beginner tutorial`_ for more information about using colcon


Running
-------

Network Node Discovery
^^^^^^^^^^^^^^^^^^^^^^
By default eprosima Fast DDS (The default DDS implementation used in ROS2 Humble) will use multicast
messages in order to find ROS nodes in the network. In some network settings this might not
work. Alternively one can also use a server running on one of the hosts for client discovery.

- Start server on one of the nodes:

..  code-block::
    fast-discovery-server -i 0

Before starting a ROS node export the following

..  code-block::
    export ROS_DISCOVERY_SERVER=<ip of host running discovery server>:11811

using ``export ROS_DOMAIN_ID=0`` one can define different ROS domains, in case multiple independent ROS
instances need to be executed.

Using the discovery server you will be able to connect to topics, but due to a current ongoing issue
in ROS2 humble, the deamon will not be able to enumerate running topics and nodes. See Section
`16.2.5.1`_ for required configuration of the ROS2 daemon.

Simulation
^^^^^^^^^^
In order to run the simulation, execute the ``robot_launch.py`` script from the orcar_webots_sim
package. Currently only `Webots`_ is supported as a simulation target. If you are on Ubuntu, it is
not necessary to manually install Webots, as it will be downloadded and installed automatically by
the Webots launcher.

..  code-block::

    ros2 launch orcar_webots_sim robot_launch.py

Support for MacOS is currently Work-in-Progress. See the `webots-ros2 complete installation guide`_.
In short, it relies on running ROS inside a Docker container and communicating over a server running
on the host providing the Webots instance.

Note: The above Guide currently only works on nightly builds of Webots 2022b.

Orcars
^^^^^^^^^^^^^^^

Currently we only implement drivers for the servo motors. Make sure that the nodes running on the
orcar are discoverable on the network. You will be able to control the robot through the topic
``/<robot_namespace>/cmd_vel``.

Tips
----
RVIZ Visualization
^^^^^^^^^^^^^^^^^^
You can use ``rviz2`` in order to display information from ROS2 topics. Alternatively, if you are on
MacOS and don't want to install a complete ROS2 distribution, you can also use `foxglove`_ .
Foxglove combines both visualization and teleop mechanisms.

ROS Bridge
^^^^^^^^^^^^^^^^^^
If DDS node discovery does not work, you can also use ROS Bridge to connect Foxglove with your ROS2 instance.
First install rosbridge on the node you wan't to connect to using Foxglove.
.. code::

   sudo apt install ros-humble-rosbridge-suite
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml


This this spawn a bridge on port ``9090``. Keep in mind that if you for example are on Docker you
will need to forward this port to the host.

Interact with robot
^^^^^^^^^^^^^^^^^^
Orcar receives `Twist`_ messages on the ``/<robot-name>/cmd_vel`` topic.
You can use the simpe gui ``rqt_robot_steering`` in order to send Twist messages to a Orcar node.

..  code-block::

    sudo apt install ros-humble-rqt-robot-steering
    rqt-robot-steering


Resources
=========
.. _common-interfaces-guide:https://github.com/ros2/common_interfaces

.. _ROS2: https://docs.ros.org/en/humble/index.html
.. _installation documentation: https://docs.ros.org/en/humble/Installation.html
.. _colcon beginner tutorial: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
.. _webots: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
.. _webots-ros2 complete installation guide: https://github.com/cyberbotics/webots_ros2/wiki/Complete-Installation-Guide
.. _Twist: https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html
.. _foxglove: https://foxglove.dev/
.. _Robostack: https://github.com/RoboStack
.. _installation guide: https://robostack.github.io/GettingStarted.html.

.. _16.2.5.1: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html#daemon-s-related-commands
