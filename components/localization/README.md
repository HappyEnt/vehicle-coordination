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