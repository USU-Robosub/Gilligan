#!/usr/bin/env python
import roslib
roslib.load_manifest('QualifyTask')
import rospy
import sys

from std_msgs.msg import UInt8, Float32
from SubMotorController.msg import MotorMessage
from Robosub.msg import ModuleEnableMsg

from time import sleep
    
class QualifyTask:

    def __init__(self):
        self.moduleEnabled = False
        self.taskThread = None
        self.controlManualPub = None
        self.depthPub = None
        self.killSwitchSub = None
        self.isEnabledSub = None
        
        self.goForwardManualMsg = MotorMessage()
        
        rospy.loginfo("Qualifying node started")
        #Startup the node
        rospy.init_node('diveAndDrive')
        
        #Set up publishers
        self.depthPub = rospy.Publisher('Simple_Depth', Float32)
        self.controlManualPub = rospy.Publisher('Motor_Control', MotorMessage)
        
        #Set up Subscribers
        self.isEnabledSub = rospy.Subscriber('Module_Enable', ModuleEnableMsg, self.moduleEnabledCallback)

        
        self.goForwardManualMsg.mask = 3
        


        rospy.spin()
        
        
    def moduleEnabledCallback(self, msg):
        rospy.loginfo("Callback")
        if (msg.Module = "Qualify" && msg.State == 1):
            if not self.taskThread:
                self.taskThread = start_new_thread(self.diveAndDrive, tuple())
                rospy.loginfo("Starting qualify task")
        else:
            if self.taskThread:
                reset()

    def diveAndDrive(self):
        #dive! :D
        sleep(3)
        self.depthPub.publish(Float32(3.0))
        sleep(10)
        
        self.goForwardManualMsg.Left = 120
        self.goForwardManualMsg.Right = 120
        self.controlManualPub.publish(self.goForwardManualMsg)
        
        sleep(60)
        start_new_thread(reset, tuple())
            
    def reset(self):
        self.taskThread.exit()
        self.taskThread = None;
        
        self.goForwardManualMsg.Left = 0
        self.goForwardManualMsg.Right = 0
        self.controlManualPub.publish(goForwardManualMsg)
        
        self.depthPub.publish(Float32(3.0))
        
if __name__ == '__main__':
    try:
        rospy.loginfo("main")
        qualify_task = QualifyTask()
    except rospy.ROSInterruptException:
        pass
    
    
