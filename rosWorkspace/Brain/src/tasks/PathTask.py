import rospy
import smach
from std_msgs.msg import String

from Robosub.msg import ModuleEnableMsg

class PathTask(smach.State):
    def __init__(self):
        super(PathTask, self).__init__(outcomes=['succeeded', 'preempted'])
        self.is_complete = False
    def execute(self, userdata):
        self.pub = rospy.Publisher('/Module_Enable', ModuleEnableMsg)
        self.sub = rospy.Subscriber('/Task_Completion', String, self.task_complete)
        msg = ModuleEnableMsg()
        msg.Module = 'PathTask'
        msg.State = True
        self.pub.publish(msg)
        for i in range(100):
            if self.is_complete:
                self.disable_task()
                return 'succeeded'
            if self.preempt_requested():
                self.disable_task()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        self.disable_task()
        return 'succeeded'
    def disable_task(self):
        msg = ModuleEnableMsg()
        msg.Module = 'PathTask'
        msg.State = False
        self.pub.publish(msg)
    def task_complete(self, msg):
        if msg.data == "PathTask":
            self.is_complete = True
