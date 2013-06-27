#!/bin/bash

set -eu

## Stage 1

roscore &

ROS_NAMESPACE=stereo
sleep 2

## Stage 2

roslaunch pgr_camera_driver camera_node_left.launch _camera_node:=left _serialnumber:=12460898 &

roslaunch pgr_camera_driver camera_node_right.launch _camera_node:=right _serial_number:=13021177 &
