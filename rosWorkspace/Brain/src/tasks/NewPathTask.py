import rospy
import smach
from std_msgs.msg import String
from SubImageRecognition.msg import ImgRecObject
from Robosub.msg import ModuleEnableMsg
from utils import turn, forward, dive, strafe

class NewPathTask(smach.State):
    def __init__(self):
        super(NewPathTask, self).__init__(outcomes=['succeeded', 'preempted', 'timedout'])
        self.is_complete=False
        self.foundObj=False
        self.timeout=100
        # TODO fix center coords
    def extendTimeout(self):
        self.timeout= self.timeout if self.timeout<30 else 30
    def objCallback(self, msg):
        self.foundObj=True
        self.extendTimeout()
        forward(0)
        self.centerOnPath(msg)
    def execute(self, userdata):
        self.objSub = rospy.Subscriber('img_rec/paths', ImgRecObject, self.objCallback)
        msg = ModuleEnableMsg()
        msg.Module = 'NewPathTask'
        msg.State = True
        self.pub.publish(msg)
        dive(4)
        while self.timeout>0:
            if self.is_complete:
                self.disable_task()
                return 'succeeded'
            if self.preempt_requested():
                self.disable_task()
                self.service_preempt()
                return 'preempted'
            forward(0.35)
            rospy.sleep(1)
            self.timeout-=1
        self.disable_task()
        return 'timedout'
    def disable_task(self):
        msg = ModuleEnableMsg()
        msg.Module = 'NewPathTask'
        msg.State = False
        self.pub.publish(msg)
    def task_complete(self, msg):
        if msg.data == "NewPathTask":
            self.is_complete = True
    def centerX(self, msg):
        if abs(msg.center_x) > 30:
            strafe(-(msg.center_x/200))
            return False
        strafe(0)
        return True
    def centerY(self, msg):
        if abs(msg.center_y) > 30:
            forward(-(msg.center_y/300))
            return False
        forward(0)
        return True
    def centerOnPath(self, msg):
        if self.centerX(msg) and self.centerY(msg):
            self.alignToPath(msg)
    def alignToPath(self, msg):
        if abs(msg.rotation)<3:
            turn(0)
            self.is_complete=True
        else:
            turn(-(msg.rotation/90))
