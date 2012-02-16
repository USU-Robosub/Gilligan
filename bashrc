# Add this to your .bashrc to auto-set ROS_IP to your own IP address. This resolves P2P connection issues :)
export ROS_IP=`/sbin/ifconfig | grep "inet addr" | head -n 1 | cut -d ':' -f 2 | cut -d ' ' -f 1`
