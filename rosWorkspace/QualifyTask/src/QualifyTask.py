#!/usr/bin/env python
import roslib
roslib.load_manifest('QualifyTask')
import rospy
import sys

from std_msgs.msg import UInt8, Float32
from SubMotorController.msg import MotorMessage
from thread import start_new_thread

from time import sleep

taskThread = None;
controlManualPub = None;
depthPub = None;
killSwitchSub = None;

goForwardManualMsg = MotorMessage()

def setup():
    rospy.loginfo("Hello, qualifying...")
    #Startup the node
    rospy.init_node('diveAndDrive')
    
    #Set up publishers
    depthPub = rospy.Publisher('Simple_Depth', Float32)
    controlManualPub = rospy.Publisher('Motor_Control', MotorMessage)
    
    #Set up Subscribers
    killSwitchSub = rospy.Subscriber('Motor_State', UInt8, killSwitchCallback)

    
    goForwardManualMsg.mask = 3
    


    rospy.spin()
    
    
def killSwitchCallback(motorState):
	rospy.loginfo("Callback")
    #if (motorState.data == 1):
        #if not taskThread:
            #taskThread = start_new_thread(diveAndDrive, tuple())
        
    #else:
        #if taskThread:
            #reset()

def diveAndDrive():
    #dive! :D
    sleep(3)
    depthPub.publish(Float32(3.0))
    sleep(10)
    
    goForwardManualMsg.Left = 120
    goForwardManualMsg.Right = 120
    controlManualPub.publish(goForwardManualMsg)
    
    sleep(60)
    start_new_thread(reset, tuple())
        
def reset():
    taskThread.exit()
    taskThread = None;
    
    goForwardManualMsg.Left = 0
    goForwardManualMsg.Right = 0    
    controlManualPub.publish(goForwardManualMsg)
    
    depthPub.publish(Float32(3.0))
    
if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
    
    
