#!/bin/bash

# This is meant to be run by cron every minute or so. It will
# restart networking if eth0 is not connected. This allows the
# computer to reconnect on eth0 without needing to be restarted

## Test One: Do we have an IP Address?

INET_ADDR=`/sbin/ifconfig | /bin/grep -A 1 'eth0' | /bin/grep 'inet addr'`

if [ "$INET_ADDR" == '' ]; then
	/etc/init.d/networking restart > /dev/null &
	echo 'restarting network' > /home/robosub/.networkWatchdogStatus
	exit
fi

## Test Two: Can we ping the gateway?

GATEWAY=`/sbin/route -n | /bin/grep "^0\.0\.0\.0" | cut -d ' ' -f 10`

RESULT=`/bin/ping -qc 4 $GATEWAY | /bin/grep '4 packets transmitted' | /bin/grep '100% packet loss'`

if [ "$RESULT" != '' ]; then
	/etc/init.d/networking restart > /dev/null &
	echo 'restarting network' > /home/robosub/.networkWatchdogStatus
	exit
fi

echo 'network up' > /home/robosub/.networkWatchdogStatus
