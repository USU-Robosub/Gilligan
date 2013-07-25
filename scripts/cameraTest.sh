#!/bin/bash

set -eu

## Stage 1

roscore &

ROS_NAMESPACE=stereo
sleep 2

## Stage 2



# Start camera drivers
echo "Start camera drivers" >&3
roslaunch pgr_camera_driver camera_node.launch _camera_node:=left _serialnumber:=13021177 _calibration_file:=stereo_intrinsics.ini &
roslaunch pgr_camera_driver camera_node.launch _camera_node:=right _serialnumber:=12460898 _calibration_file:=stereo_intrinsics.ini &


