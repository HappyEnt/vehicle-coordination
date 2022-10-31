=============
Orcar on ROS2
=============

Installation
------------

Dependencies
^^^^^^^^^^^^

Building
^^^^^^^^
Before trying to build the project make sure that you have a working ros installation.  Currently
supported is the humble release of ROS2.  See the official ROS2 `installation documentation`_ for instructions.

ROS2 works best on Ubuntu, for installation on MacOS it is therefore recommended to either use a Ubuntu Docker
container or a Virtual Machine.

In order to build the project, simply run colcon build from the workspace folder.
Afterwards you will have to source install/local_setup.bash so the newly build packages
can be found by ROS.

..  code-block::

    cd ~/vehicle-coordination/components/ros
    colcon build
    source install/local_setup.bash

Look into the `colcon beginner tutorial`_ for more information about using colcon

Running
-------

Simulation
^^^^^^^^^^
In order to run the example simulation simply run

..  code-block::

    ros2 launch orcar_webots_sim robot_launch.py

Tips
----
RVIZ Visualization
^^^^^^^^^^^^^^^^^^
You can use ``rviz2`` in order to display information from ROS2 topics.


Resources
=========
.. _common-interfaces-guide:https://github.com/ros2/common_interfaces


.. _installation documentation: https://docs.ros.org/en/humble/Installation.html
.. _colcon beginner tutorial: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
