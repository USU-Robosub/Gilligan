#!/bin/bash

set -eu

## Stage 1

##roscore &

sleep 2

## Stage 2

# Start camera drivers
echo "Start camera drivers" 

roslaunch pgr_camera_driver camera_node.launch camera_node:=left camera_name:='stereo/left' serialnumber:=13021177 calibration_file:='file:///opt/robosub/rosWorkspace/pgr_camera_driver/stereo_intrinsics.ini' &
roslaunch pgr_camera_driver camera_node.launch camera_node:=right camera_name:='stereo/right' serialnumber:=12460898 calibration_file:='file:///opt/robosub/rosWorkspace/pgr_camera_driver/stereo_intrinsics.ini' &



##Run stereo image processing

ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc &
