#!/usr/bin/env python
import roslib
roslib.load_manifest('QualifyTask')
import rospy
import sys

from std_msgs.msg import UInt8, Float32
from SubMotorController.msg import MotorMessage
from thread import start_new_thread
from threading import Thread

from time import sleep

FANCY_SLEEP_SLICE = 0.5
TARGET_DEPTH = 2.0
SPEED_LEFT = 150
SPEED_RIGHT = 200
TIME_DELAY = 0
TIME_DIVE = 4
TIME_FORWARD = 100

class QualifyTask:

    def __init__(self):
        self.seenKilledYet = False
        self.controlManualPub = None;
        self.depthPub = None;
        self.killSwitchSub = None;
        self.isEnabled = False 
        self.goForwardManualMsg = MotorMessage()
        
        rospy.loginfo("Hello, qualifying...")
        #Startup the node
        rospy.init_node('diveAndDrive')
        
        #Set up publishers
        self.depthPub = rospy.Publisher('Target_Depth', Float32)
        self.controlManualPub = rospy.Publisher('Motor_Control', MotorMessage)
        
        #Set up Subscribers
        self.killSwitchSub = rospy.Subscriber('Motor_State', UInt8, self.killSwitchCallback)

        self.goForwardManualMsg.mask = 3

        self.reset()

        rospy.spin()
        
        
    def killSwitchCallback(self, motorState):
        rospy.loginfo("Kill switch is currently {}".format(motorState.data))
        if not self.seenKilledYet and motorState.data == 1:
            return
        elif not self.seenKilledYet and motorState.data == 0:
            self.seenKilledYet = True
        if motorState.data == 1:
            if not self.isEnabled:
                self.isEnabled = True
                thread = Thread(target=self.diveAndDrive)
                thread.start()
                rospy.loginfo("Starting qualify task")
        else:
            self.isEnabled = False

    def diveAndDrive(self):
        #dive! :D
        if self.fancySleep(TIME_DELAY): return
        rospy.loginfo("diving...")
        self.depthPub.publish(Float32(TARGET_DEPTH))
        if self.fancySleep(TIME_DIVE): return
        rospy.loginfo("going foward...")
        self.goForwardManualMsg.Left = SPEED_LEFT
        self.goForwardManualMsg.Right = SPEED_RIGHT
        self.controlManualPub.publish(self.goForwardManualMsg)
        if self.fancySleep(TIME_FORWARD): return
        self.reset()
    
    def fancySleep(self, timeLeft):
        while self.isEnabled and timeLeft:
            timeLeft -= FANCY_SLEEP_SLICE
            sleep(FANCY_SLEEP_SLICE)
        if self.isEnabled:
            return False  # Don't die yet
        else:
            self.reset()
            return True   # Sacrifice our bits
    
    def reset(self):
        self.goForwardManualMsg.Left = 0
        self.goForwardManualMsg.Right = 0
        self.controlManualPub.publish(self.goForwardManualMsg)
        
        self.depthPub.publish(Float32(0.0))
        
        self.isEnabled = False
        self.seenKilledYet = False
        
        rospy.loginfo("everything is reset")


if __name__ == '__main__':
    qualify_task = QualifyTask()

