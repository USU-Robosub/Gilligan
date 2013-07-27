#!/usr/bin/env python
import roslib
roslib.load_manifest('QualifyTask')
import rospy
import sys

from std_msgs.msg import UInt8, Float32
from SubMotorController.msg import MotorMessage
from Robosub.msg import HighLevelControl
from Robosub.msg import ModuleEnableMsg
from thread import start_new_thread
from threading import Thread

from time import sleep

FANCY_SLEEP_SLICE = 0.5
TARGET_DEPTH = 2.0
SPEED_LEFT = 150
SPEED_RIGHT = 200
FORWARD_SPEED = .8
TIME_DELAY = 0
TIME_DIVE = 4
TIME_FORWARD = 100

class QualifyTask:

    def __init__(self):
        self.seenKilledYet = False
        self.controlManualPub = None
        self.depthPub = None
        self.killSwitchSub = None
        self.isEnabled = False 
        self.goForwardManualMsg = MotorMessage()
        self.moduleEnableSub = None
        self.goForwardManualMsg.mask = 3
        self.goForwardMsg = HighLevelControl()
        self.diveMsg = HighLevelControl()
        self.highLevelMotionPub = None
        

        #Startup the node
        rospy.init_node('QualifyTask')
        
        #Set up publishers
        self.depthPub = rospy.Publisher('Target_Depth', Float32)
        self.controlManualPub = rospy.Publisher('Motor_Control', MotorMessage)
        
        #self.highLevelMotionPub = rospy.Publisher('High_Level_Motion', HighLevelControl)

        
        #Set up Subscribers
        self.moduleEnableSub = rospy.Subscriber('Module_Enable', ModuleEnableMsg, self.moduleEnableCallback)

        rospy.spin()
        
        
    def moduleEnableCallback(self, msg):
        rospy.loginfo("{} qStatus: {}".format(msg.Module, msg.State))
        if msg.Module == "QualifyTask":
            if msg.State == True and self.isEnabled == False:
                self.isEnabled = True
                thread = Thread(target=self.diveAndDrive)
                thread.start()
                rospy.loginfo("Starting qualify task")
            elif msg.State == False and self.isEnabled == True:
            	rospy.loginfo("QualifyTask Deactivated")
                self.isEnabled = False
                self.reset()
            else:
                rospy.loginfo("I'm already %i, don't send me %i", self.isEnabled, msg.State)

    def diveAndDrive(self):
        #dive! :D
        if self.fancySleep(TIME_DELAY): return
        rospy.loginfo("diving...")
        #Old method
        self.depthPub.publish(Float32(TARGET_DEPTH))
        #High level method
        #self.diveMsg.Direction = "Depth"
        #self.diveMsg.MotionType = "Command"
        #self.diveMsg.Value = TARGET_DEPTH
        #self.highLevelMotionPub.publish(self.diveMsg)
        if self.fancySleep(TIME_DIVE): return
        rospy.loginfo("going foward...")
        self.goForwardManualMsg.Left = SPEED_LEFT
        self.goForwardManualMsg.Right = SPEED_RIGHT
        #Old method
        self.controlManualPub.publish(self.goForwardManualMsg)
        #High level method
        #self.goForwardMsg.Direction = "Forward"
        #self.goForwardMsg.MotionType = "Command"
        #self.goForwardMsg.Value = FORWARD_SPEED
        #self.highLevelMotionPub.publish(self.goForwardMsg)
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
        self.isEnabled = False
        rospy.loginfo("everything is reset")


if __name__ == '__main__':
    qualify_task = QualifyTask()

