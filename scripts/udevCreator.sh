#/bin/bash
#The first argument should be the usb device location to search for, ie ttyUSB0
#The second argument should be the new location
#
# Place the output of this script into a udev/rules.d/ file
#It is a good idea to set the file at around 10, ie 10-descriptivename.rules in /etc/udev/rules.d/
if [ ! -n "$1" ] || [ ! -n "$2" ]
then
	echo "Usage: `basename $0` devName newDevName"
	exit
fi

EXEC=`udevadm info -a -p $(udevadm info -q path -n $1) | egrep -i "ATTRS{serial}|ATTRS{idVendor}|ATTRS{idProduct}" -m 3 | gawk '{ print $1}' | sed 's/ATTRS/SYSFS/;' | sed 'N;s/\n/, /;' | sed 'N;s/\n/, /;'`

echo "BUS==\"usb\", $EXEC, NAME=\"$2\""

