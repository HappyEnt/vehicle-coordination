The Localization Module
=======================

Usage
-----

All following commands assume to be executed from ``components/localization`` as the working directory.

Make sure all dependencies from ``requirements.txt`` are installed.

Entry point of this application is ``src/main.py``. To run it with a UWB-board connected over USB use the ``--port`` command-line option for example on macos:

.. code-block::

    python src/main.py --port /dev/tty.usb...

Alternatively, we can also evaluate output from a device, that has been previously written to a file with:

.. code-block::

    screen -L /dev/tty.usb... 115200

To do this, run:

.. code-block::

    python src/main.py --file <screenlog_file>


UWB Calibration
---------------

One factor of improving the accuracy of UWB ranging is calibration. Which in this case is finding the correct RX and TX delays for each board. There is some time between the recording of the transmission or reception timestamp and the signal leaving or arriving at the antenna, respectively. Those times may differ from device to device.

The ranging logic is able to adjust to RX and TX delays. There is also a script in ``calibrate.py`` which uses an optimization to find the delays for each device. Adjust the ``real_positions`` variable as a ground truth and the ``screenlog_file`` to a file with a recording from a board.

Source Files Explanation
------------------------

A small explanation on what file of the localization module does what.
Remember that our whole vehicle system generally consists of three parts: **Ranging**, **Localization** and **Coordination**.

``main.py``
    Starts localization and ranging.
``data.py``
    Definition of data types for ranging and localization.  

Ranging
^^^^^^^^

``twr.py``
    Heart of the Ranging.  
``calibrate.py``
    Used for the calibration of UWB modules.  
``config.py``
    Config entries for ranging.  

Localization
^^^^^^^^^^^^^

``localization.py``
    Heart of the localization. Contains all logic of the localization and communication with coordination and ranging.
``car_config.ini``
    Holds information about a car/vehicle.  
``config.ini``
    Holds information about the operation area.  
``interface_pb2.py``
    Generated file from python grpcio module. Used for GRPC communication (communication from localization to coordination).
``interface_pb2_grpc.py``
    Generated file from python grpcio module. Used for GRPC communication (communication from localization to coordination).

About Ranging
-------------

The main idea of UWB ranging is to measure the ToF between two antennas and thereby obtaining the distance.

About Particle Filters
----------------------