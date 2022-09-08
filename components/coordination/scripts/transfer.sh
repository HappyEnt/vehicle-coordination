#!/bin/bash

HOST=$1
DIR=${2:-/home/pi/bin}

scp target/armv7-unknown-linux-gnueabihf/release/{picar-coordination,camera-interface} $HOST:$DIR
