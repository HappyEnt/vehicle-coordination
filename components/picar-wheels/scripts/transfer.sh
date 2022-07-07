#!/bin/bash

HOST=$1
DIR=${2:-/home/pi/bin}

scp target/armv7-unknown-linux-gnueabihf/release/{picar,picar-daemon} $HOST:$DIR
scp config/picar.service $HOST:/home/pi/.config/systemd/user
