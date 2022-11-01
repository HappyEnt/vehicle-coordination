=============
Orcar on ROS2
=============

Here provided is a skeleton for running Orcar on `ROS2`_. Robot Operating System, short ROS, is a middleware
that provides among other things primitives for communication between components that constitute a robot, like sensors, navigation algorithms, ...
Currently implemented is localization using a variety of particle filter algorithms and a simulation with a virtual model of our Orcar.

Installation
------------

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

    sudo apt install libgsl-dev boost protobuf-compiler

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

Physical Orcars
^^^^^^^^^^^^^^^

Currently not supported, the following essential components are missing:

- ``Driver for the Servo motors``
- ``Interface for the UWB transceiver``


Tips
----
RVIZ Visualization
^^^^^^^^^^^^^^^^^^
You can use ``rviz2`` in order to display information from ROS2 topics.

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
