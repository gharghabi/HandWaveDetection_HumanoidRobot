#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#Devices
 kill -9 ` ps -ef | grep kinect.sh | cut -d "a" -f 1`
gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"
