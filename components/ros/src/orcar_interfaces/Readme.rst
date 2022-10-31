=============
Orcar Interfaces
=============

This package defines resources that are used for interfacing between the different orcar components.

We use both ROS2 Messages and Protobuf. Protobuf definitions are used for serializing and deserializing data
for sending over radio, whereas ROS2 Messages are used for exchanging data over `DDS`_.

``msg/``
  ``RadioPacket.msg``
   Just a raw byte array.
  ``TaggedRadioPacket.msg``
   A ``RadioPacket`` together with the mac of the sender and the TWR distance that was measured
   during the message exchange.
``protobuf/``
  ``Particles.proto``
   definition of ``Particle`` and ``Particles`` types.


.. _DDS: https://design.ros2.org/articles/ros_on_dds.html
