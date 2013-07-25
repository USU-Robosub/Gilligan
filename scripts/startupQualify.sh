#!/bin/bash

set -eu

LOGFILE=/home/robosub/roslog.log
PIDFILE=/home/robosub/rossession.pid

exec 3>&1
echo "Writing to $LOGFILE"

exec 1>$LOGFILE
#exec 2>&1 

echo "---------------------------"
echo "SESSION: $( date )"

## Stage 1


roscore &

sleep 2

## Stage 2






# Start the sensor board
echo "Start the sensor board" >&3
rosrun SubSensorController SubSensorController /dev/controller_sensor &  

# Start the imu
echo "Start the IMU" >&3
#rosrun SubAttitudeResolver SubAttitudeResolver /dev/controller_Imu &
rosrun SubImuController SubImuController /dev/controller_Imu > /home/robosub/SubImuController.log  &  

# Start the motor controllers
echo "Start the motor controller"
roslaunch SubMotorController MotorController.launch &


# Start the high level contorl
#rosrun SubMotorController SubHighLevelMotorController &  


echo "Start depth translator" >&3
# Translation from ticks to actual depth
rosrun SubTranslators DepthTranslator &  

#echo "Start state machine" >&3
#rosrun SubStateMachine SubStateMachine.py &  
#rosrun GateTask GateTask &  
#rosrun PathTask PathTask &  
#rosrun BuoyTask BuoyTask RED YELLOW &  

echo "Start depth controller"
rosrun SubDepthController SubDepthController &

# Simple Heading Controller maintains a target heading
#rosrun subSim simpleHeading &  

sleep 4

## Stage 3


echo Starting sensor bag
# Save sensor data in a bag
rosbag record -O /home/robosub/bags/sensors.`date +%Y%m%d%H%M`.bag Calibrate_Depth Computer_Cur_Volt Controller_Box_Temp Error_Log IMU_Attitude IMU_Accel_Debug IMU_Gyro_Debug Mobo_Temp Motor_Control Motor_State Motor_Current Pressure_Data Sub_Depth Target_Depth Water_Detected Points_Of_Interest &  

echo "Calibrating depth to zero">&3
# Calibrate the current pressure as 0
rostopic pub /Calibrate_Depth std_msgs/Float32 -1 -- 0.0 &
disown $!

sleep 2

# Start Qualify
echo "Starting jimmy rigged qualify node"
rosrun QualifyTask QualifyTask.py &

exec 1>&3
jobs -p > $PIDFILE

echo "Runnning processes:"
jobs -l

## Complete
echo "\nStartup Complete!"
echo "To tail the log file run 'roslog'"
echo "To kill all processes run 'killRos'"
