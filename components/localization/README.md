# The Localization Module

## Usage

Make sure all dependencies from `requirements.txt` are installed.

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
