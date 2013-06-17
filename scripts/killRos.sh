#!/bin/bash

PIDFILE=/home/robosub/rossession.pid


if [ -e $PIDFILE ]
then
 kill `cat $PIDFILE`
 rm $PIDFILE
 sleep 5

 else
 echo "Cannot find $PIDFILE!"
fi

#Check if there are any ros processes
 PIDS=`ps ax | grep -c ros`
if [[ $PIDS>1 ]]
 then
  echo "It seems like there are some running processes still. Run 'ps ax | grep
  ros' for mor info"
 fi

echo "Done"
