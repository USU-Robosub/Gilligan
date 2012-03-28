#!/bin/bash

roscore &

sleep 2

# cameras driver
roslaunch SubCameraDriver cameras.launch &

sleep 1

# republish cameras as compressed for recording
/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=left/image_raw compressed out:=left/image_compressed &
/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=right/image_raw compressed out:=right/image_compressed &

# image recognition
/opt/robosub/rosWorkspace/SubImageRecognition/src/image_recognition.py &

sleep 1

# republish image recognition as compressed for viewing remotely
/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=forward_camera/image_raw compressed out:=forward_camera/image_compressed &
/opt/ros/diamondback/stacks/image_common/image_transport/bin/republish raw in:=downward_camera/image_raw compressed out:=downward_camera/image_compressed &

# save compressed cameras in a bag
rosbag record -O cameras.`date +%Y%m%d%H%M`.bag left/image_compressed left/image_compressed/compressed right/image_compressed right/image_compressed/compressed &

