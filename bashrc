# Add this to your .bashrc to auto-set ROS_IP to your own IP address. This resolves P2P connection issues :)
export ROS_IP=`/sbin/ifconfig | grep "inet addr" | head -n 1 | cut -d ':' -f 2 | cut -d ' ' -f 1`

# This adds the extra package paths needed on submarine.bluezone.usu.edu
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/opt/robosub/rosWorkspace:/opt/camera_umd"
