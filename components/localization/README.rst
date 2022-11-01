Usage
-----

Installation
^^^^^^^^^^^^
All following commands assume to be executed from ``components/localization`` as the working directory.

Make sure all dependencies from ``requirements.txt`` are installed (Normally with pip). On a RaspberryPi, pip is not working really well, to install the needed libraries, use the following commands:  

.. code-block::

    pip uninstall grpcio
    sudo apt-get install python3-protobuf
    sudo apt-get install python3-grpcio

    sudo apt-get install libopenjp2-7
    pip uninstall matplotlib
    sudo apt-get install python3-matplotlib
    pip uninstall scipy
    sudo apt install python3-scipy

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
^^^^^^^^^^^^^^^
One factor of improving the accuracy of UWB ranging is calibration. Which in this case is finding the correct RX and TX delays for each board. There is some time between the recording of the transmission or reception timestamp and the signal leaving or arriving at the antenna, respectively. Those times may differ from device to device.

The ranging logic is able to adjust to RX and TX delays. There is also a script in ``calibrate.py`` which uses an optimization to find the delays for each device. Adjust the ``real_positions`` variable as a ground truth and the ``screenlog_file`` to a file with a recording from a board.

Configuring vehicles and operation area
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For the particle filter to work, the operation area needs to be inserted in the file ``config.ini``. If you are still using the camera-server component,  
this file should be the same as the ``config.ini`` in the camera-server component (see README of camera-server component).

If you are using multiple vehicles, each vehicle needs an **id**. The id is stored in the file ``car_config.ini`` (as well as size of the vehicle).   
If you are still using the camera-server, the **id** in the file ``car_config.ini`` should be the same as the aruco-marker attached to the vehicle.  
For more information about the markers, look into the README from the camera-server component.

Source Files Explanation
------------------------

A small explanation on what file of the localization module does what.
Remember that our whole vehicle system generally consists of three parts: **Ranging**, **Localization** and **Coordination**. This component mainly covers the parts **Ranging** and **Localization**.

Ranging:
^^^^^^^^
In folder **ranging**:  

``AbstractRangingNode.py``
    Abstract class for ranging class.
``SerialRangingNode.py``
    Extension of the `RangingNode` to work with a UWB board connected over a serial connection.
``DumpFileRangingNode.py``
    Extension of the `RangingNode` to work with a dump file.
``twr.py``
    Heart of the Ranging.  
``calibrate.py``
    Used for the calibration of UWB modules.  

Localization:
^^^^^^^^^^^^^
In folder **filtering**:

``AbstractLocalizationNode.py``
    Abstract class for all filters.
``BaseParticleNode.py``
    Superclass for all filters.
``ClassicAllAtOnce.py``
    Particle filter which uses all measurements from all nodes to do a single particle filter step.
``ClassicParticleNode.py``
    Particle filter which only uses one distance measurement to update the particles.
``GridParticleNode.py``
    Particle filter, which uses a grid for particle positions.
``car_config.ini``
    Holds information about a car/vehicle (size, id).   
``config.ini``
    Holds information about the operation area.  

Other:
^^^^^^
In this folder:

``main.py``
    Starts localization and ranging.
``data.py``
    Definition of data types for ranging and localization.  
``interface_pb2.py``
    Generated file from python grpcio module (proto definition). Used for GRPC communication (communication from localization to coordination).
``interface_pb2_grpc.py``
    Generated file from python grpcio module (proto definition). Used for GRPC communication (communication from localization to coordination).
``decorators.py``
    Function decorators, useful for evaluation.
``scripts/track_estimates.py``
    Shows the estimates from the localization in a matplotlib plot. It uses the data from the server/camera-server.
``decorators.py``
    Holds some decorators, which can be used for evaluation.

About Ranging
-------------
The main idea of UWB ranging is to measure the ToF between two antennas and thereby obtaining the distance.

About Particle Filters
----------------------
With Particle Filters, we can approximate the position of a car, using the distance measurements from the UWB ranging. This is done using so called "particles" for each car, which represent a possible position of the car.
These particles are then updated with new measurements (from UWB Ranging and odometry data). We can then calculate an estimated position from these particles.
