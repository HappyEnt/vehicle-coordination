# The Localization Module

## Usage

Make sure all dependencies from `requirements.txt` are installed (Normally with pip). On a RaspberryPi, pip is not working really well, to install the needed libraries, use the following commands:  
```
pip uninstall grpcio
sudo apt-get install python3-protobuf
sudo apt-get install python3-grpcio

sudo apt-get install libopenjp2-7
pip uninstall matplotlib
sudo apt-get install python3-matplotlib
pip uninstall scipy
sudo apt install python3-scipy
```

Entry point of this application is `src/main.py`. To run it with a UWB-board connected over USB use the `--port` command-line option for example on macos:
```
python src/main.py --port /dev/tty.usb...
```
Alternatively, we can also evaluate output from a device, that has been previously written to a file with:
```
screen -L /dev/tty.usb... 115200
```
To do this, run:
```
python src/main.py --file <screenlog_file>
```

### Source Files Explanation
A small explanation on what file does what.  
Remember that our whole vehicle system generally consits of three parts: **Ranging**, **Localization** and **Coordination**.
This folder covers the two parts **Ranging** and **Localization**.

#### Ranging:
```twr.py``` Heart of the Ranging.  
```calibrate.py``` Used for the calibration of UWB modules.  
```data.py``` Definition of data types for ranging.  
```config.py``` Config entries for ranging.  

```main.py``` Starts localization and ranging.  

#### Localization:  
```localization.py``` Heart of the localization. Contains all logic of the localization and communication with coordination and ranging.
```car_config.ini``` Holds information about a car/vehicle.  
```config.ini```  Holds information about the operation area.  
```interface_pb2.py``` Generated file from python grpcio module. Used for GRPC communication (communication from localization to coordination).  
```interface_pb2_grpc.py``` Generated file from python grpcio module. Used for GRPC communication (communication from localization to coordination).  

#### Other:
```track_estimates.py``` Shows the estimates from the localization in a matplotlib plot. It uses the data from the server/camera-server.  