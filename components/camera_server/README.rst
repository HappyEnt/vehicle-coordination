
The camera server is an alternative to the ranging and localization. It can track the vehicles using aruco-markers (generator for the markers: https://chev.me/arucogen/), which represent a number and can be detected with a camera and OpenCV.  
We generally use this module for our evaluation and currently also for communication between the vehicles. Furthermore, currently the coordination only works with the camera. For this to work, this module provides a webserver.  
In our work, we used a special camera from IDS, which can be adapted and used with code.  

Installation
------------

Use camera from IDS
^^^^^^^^^^^^^^^^^^^
Select your os and download IDS Software Suite from:
https://de.ids-imaging.com/download-details/AB01206.html?os=linux_arm&version=v8&bus=32&floatcalc=hard
(We used IDS Software Suite Version 4.95 as debian packages)

Install IDS Software Suite.
Installation guide from IDS: 
https://de.ids-imaging.com/files/downloads/ids-software-suite/readme/readme-ids-software-suite-linux-4.95.0_EN.html#installation

You can also use another USB camera (controlled without code). To do this, use the file ``TrackerCamera.py`` instead of ``TrackerPyUEyeLinux.py``.

Install python libraries
^^^^^^^^^^^^^^^^^^^^^^^^

Install all other libraries in the requirements.txt file (normally with pip).

.. code-block::

    pip install -r requirements.txt

Usage
-----

Before running the system, create an area (physically) with four aruco markers:

- The marker with ID 0 has to be on the top-left corner.

- The marker with ID 1 has to be on the top-right corner.  

- The marker with ID 2 has to be on the bottom-right corner.  

- The marker with ID 3 has to be on the bottom-left corner.  

Measure the area's width and length. Length is from ID 0 to ID 1 (or from 3 to 2, x-axis) and width is from ID 0 to ID 3 (or from 1 to 2, y-axis).  

Put the measured length and width in the file ``config.ini``.

Clone this repository and run:

.. code-block::

    python3 webserver_and_camera.py


If you want to run the program without a screen, run the program with "--headless":

.. code-block::

    python3 webserver_and_camera.py --headless

Remember to save the IP-address and port of the server, as it is also used for the other components.

Source Files Explanation
------------------------

``CameraConverter.py`` Converts the camera data to coordinates.  

``TrackerPyUEyeLinux.py`` Code to read the camera data from an IDS camera.  

``TrackerCamera.py`` Same as ``TrackerPyUEyeLinux.py``, but it uses a normal camera (not an IDS camera) as input.

``config.ini`` Holds the size of the area.  

``flask_server.py`` Code for the webserver.

``webserver_and_camear.py`` Starts the webserver and camera and deals with communication between them.
