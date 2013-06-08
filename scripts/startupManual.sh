#!/bin/bash

set -eu


LOGFILE=rossession.log
PIDFILE=rossession.pid

## Stage 1

roscore  &
echo $! > $PIDFILE

sleep 2

## Stage 2

# Start camera drivers
roslaunch pgr_camera_driver camera_node.launch camera_node:=camera_left  serial_number:=12460898 &
echo $! >> $PIDFILE
#roslaunch pgr_camera_driver camera_node.launch camera_node:=camera_right serial_number:=13021177 &
#echo $! >> $PIDFILE
roslaunch SubCameraDriver camera.launch &
echo $! >> $PIDFILE

# Start the sensor board
rosrun SubSensorController SubSensorController /dev/controller_sensor &
echo $! >> $PIDFILE

# Start the imu
#rosrun SubAttitudeResolver SubAttitudeResolver /dev/controller_Imu &
rosrun SubImuController SubImuController /dev/controller_Imu > /home/robosub/SubImuController.log &
echo $! >> $PIDFILE

# Start the motor controllers
rosrun SubMotorController SubMotorController &
echo $! >> $PIDFILE

# Start the high level contorl
#rosrun SubMotorController SubHighLevelMotorController &

# Start the moboTemp module
#rosrun moboTemp moboTemp &

# Translation from ticks to actual depth
rosrun SubTranslators DepthTranslator  &
echo $! >> $PIDFILE

#rosrun SubStateMachine SubStateMachine.py &
#rosrun GateTask GateTask &
#rosrun PathTask PathTask &
#rosrun BuoyTask BuoyTask RED YELLOW &

# Simple Depth Controller maintains a target depth
rosrun subSim simpleDepth &
echo $! >> $PIDFILE

# Simple Heading Controller maintains a target heading
#rosrun subSim simpleHeading &

sleep 4

## Stage 3

# Republish cameras as compressed for recording
/opt/ros/fuerte/stacks/image_common/image_transport/bin/republish raw in:=camera_left/image_raw compressed out:=camera_left/image_compressed &
echo $! >> $PIDFILE
#/opt/ros/fuerte/stacks/image_common/image_transport/bin/republish raw in:=camera_right/image_raw compressed out:=camera_right/image_compressed &
#echo $! >> $PIDFILE
/opt/ros/fuerte/stacks/image_common/image_transport/bin/republish raw in:=image_raw compressed out:=image_compressed &
echo $! >> $PIDFILE

# Image recognition
/opt/robosub/rosWorkspace/SubImageRecognition/bin/ImageRecognition &
echo $! >> $PIDFILE

# Dive Alarm
#rosrun SubDiveAlarm SubDiveAlarm &

sleep 2

## Stage 4

# Republish image recognition as compressed for viewing remotely
/opt/ros/fuerte/stacks/image_common/image_transport/bin/republish raw in:=forward_camera/image_raw compressed out:=forward_camera/image_compressed &
echo $! >> $PIDFILE
/opt/ros/fuerte/stacks/image_common/image_transport/bin/republish raw in:=downward_camera/image_raw compressed out:=downward_camera/image_compressed &
echo $! >> $PIDFILE

# Save compressed cameras and resulting recognition info in a bag
#rosbag record -O /home/robosub/bags/cameras.`date +%Y%m%d%H%M`.bag left/image_compressed left/image_compressed/compressed right/image_compressed right/image_compressed/compressed image_recognition/forward/buoys image_recognition/forward/gate image_recognition/downward/orange_rectangles &

# Save sensor data in a bag
#rosbag record -O /home/robosub/bags/sensors.`date +%Y%m%d%H%M`.bag Calibrate_Depth Computer_Cur_Volt Controller_Box_Temp Error_Log IMU_Attitude IMU_Raw Mobo_Temp Motor_Control Motor_State Pressure_Data Sub_Depth Target_Depth Water_Detected Points_Of_Interest &

# Calibrate the current pressure as 0
rostopic pub /Calibrate_Depth std_msgs/Float32 -1 -- 0.0 &
echo $! >> $PIDFILE

sleep 2

## Complete

echo 'Startup Complete!'
echo "Logging to ${LOGFILE}"
echo "To kill all processes run 'kill \`cat ${PIDFILE}\`'"
