#!/usr/bin/env python
import roslib
roslib.load_manifest('QualifyTask')
import rospy
import sys

from std_msgs.msg import UInt8, Float32
from SubMotorController.msg import MotorMessage
from thread import start_new_thread

from time import sleep
    
class QualifyTask:

    def __init__(self):
        self.taskThread = None;
        self.controlManualPub = None;
        self.depthPub = None;
        self.killSwitchSub = None;
        
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
        


        rospy.spin()
        
        
    def killSwitchCallback(self, motorState):
        rospy.loginfo("Callback")
        if (motorState.data == 1):
            if not self.taskThread:
                self.taskThread = start_new_thread(self.diveAndDrive, tuple())
                rospy.loginfo("Starting qualify task")
        else:
            if self.taskThread:
                self.reset()

    def diveAndDrive(self):
        #dive! :D
        rospy.loginfo("diving...")
        sleep(3)
        self.depthPub.publish(Float32(5.0))
        sleep(10)
        rospy.loginfo("going foward...")
        self.goForwardManualMsg.Left = 110
        self.goForwardManualMsg.Right = 120
        self.controlManualPub.publish(self.goForwardManualMsg)
        
        sleep(60)
        start_new_thread(self.reset, tuple())
            
    def reset(self):
        self.taskThread.exit()
        self.taskThread = None;
        
        self.goForwardManualMsg.Left = 0
        self.goForwardManualMsg.Right = 0
        self.controlManualPub.publish(self.goForwardManualMsg)
        
        self.depthPub.publish(Float32(3.0))
        
if __name__ == '__main__':
    try:
        rospy.loginfo("main")
        qualify_task = QualifyTask()
    except rospy.ROSInterruptException:
        pass
    
    
