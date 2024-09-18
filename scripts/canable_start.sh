#!/bin/bash
# Script from https://github.com/CrossTheRoadElec/Phoenix5-Linux-Example.git

interface=can0
if [ $# -gt 0 ]; then
    interface=$1
fi

sudo ip link set $interface type can bitrate 1000000
sudo ip link set $interface up
sudo ip link set $interface txqueuelen 1000