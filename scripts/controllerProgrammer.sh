#/bin/bash

if [ ! -n "$1" ] || [ ! -n "$2" ]
then
	echo "Usage: `basename $0` [SerialPort] [FileName]"
	exit
fi

EXEC=`avrdude -pm168 -P $1 -carduino -b 19200 -e -U flash:w:$2`
