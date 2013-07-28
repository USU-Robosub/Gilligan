import rospy
import smach
from std_msgs.msg import String

from Robosub.msg import ModuleEnableMsg
from utils import move
from SubImageRecognition.msg import ImgRecObject

class BuoyTask(smach.State):
    def __init__(self):
        super(BuoyTask, self).__init__(outcomes=['succeeded', 'preempted'])
        self.is_complete = False
        self.buoy_seen = False
        self.enabled = False
        self.BUOY_DEPTH = 7.0
        
        #Subscribe and publish
        self.pub = rospy.Publisher('/Module_Enable', ModuleEnableMsg)
        self.sub = rospy.Subscriber('/Task_Completion', String, self.task_complete)
    def execute(self, userdata):
        move('Depth', 'Command', self.BUOY_DEPTH)
        
        #Begin looking for a red buoy
        self.found_buoy_sub = rospy.Subscriber('/img_rec/buoys/red', ImgRecObject, self.enable_task_cb)
        for i in range(100):
            if self.is_complete:
                rospy.loginfo('complete!')
                self.disable_task()
                return 'succeeded'
            if self.preempt_requested():
                self.disable_task()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        self.disable_task()
        rospy.loginfo("foo")
        return 'succeeded'
    def disable_task(self):
        msg = ModuleEnableMsg()
        msg.Module = 'BuoyTask'
        msg.State = False
        self.enabled = False
        self.pub.publish(msg)
    def enable_task_cb(self, msg):
        if not self.enabled:
            msg = ModuleEnableMsg()
            msg.Module = 'BuoyTask'
            msg.State = True
            self.enabled = True
            rospy.loginfo("Sending Module_Enable message: %s, %i", msg.Module, msg.State)
            self.pub.publish(msg)
    
    def task_complete(self, msg):
        self.is_complete = True