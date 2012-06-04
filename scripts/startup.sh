#!/bin/bash

## Stage 1

roscore &

sleep 2

## Stage 2

# Cameras driver
roslaunch SubCameraDriver camera.launch &

# Start the sensor board
rosrun SubSensorController SubSensorController /dev/controller_sensor &

# Start the imu
rosrun SubAttitudeResolver SubAttitudeResolver /dev/controller_Imu &

# Start the motor controllers
rosrun SubMotorController SubMotorController &

# Start the moboTemp module
rosrun moboTemp moboTemp &

# Translation from ticks to actual depth
rosrun SubTranslators DepthTranslator &

# Simple Depth Controller maintains a target depth
rosrun subSim simpleDepth &

# Simple Heading Controller maintains a target heading
rosrun subSim simpleHeading &

sleep 2

## Stage 3

# Republish cameras as compressed for recording
/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=left/image_raw compressed out:=left/image_compressed &
#/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=right/image_raw compressed out:=right/image_compressed &

# Image recognition
/opt/robosub/rosWorkspace/SubImageRecognition/src/image_recognition.py &

# Dive Alarm
rosrun SubDiveAlarm SubDiveAlarm &

# Calibrate the current pressure as 0
rostopic pub /Calibrate_Depth std_msgs/Float32 -1 -- 0.0 &

sleep 2

## Stage 4

# Republish image recognition as compressed for viewing remotely
/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=forward_camera/image_raw compressed out:=forward_camera/image_compressed &
#/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=downward_camera/image_raw compressed out:=downward_camera/image_compressed &

# Save compressed cameras and resulting recognition info in a bag
rosbag record -O /home/robosub/bags/cameras.`date +%Y%m%d%H%M`.bag left/image_compressed left/image_compressed/compressed right/image_compressed right/image_compressed/compressed image_recognition/forward/buoys image_recognition/forward/gate image_recognition/downward/orange_rectangles &

# Save sensor data in a bag
rosbag record -O /home/robosub/bags/sensors.`date +%Y%m%d%H%M`.bag Calibrate_Depth Computer_Cur_Volt Controller_Box_Temp Error_Log IMU_Attitude IMU_Raw Mobo_Temp Motor_Control Motor_State Pressure_Data Sub_Depth Target_Depth Water_Detected Points_Of_Interest &

sleep 2

## Complete

echo 'Startup Complete!'
