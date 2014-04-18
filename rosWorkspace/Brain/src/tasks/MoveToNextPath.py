import smach
import rospy
from utils import move
from SubImageRecognition.msg import ImgRecObject
from utils import turn, forward, strafe

class MoveToNextPath(smach.State):
    def __init__(self):
        super(MoveToNextPath, self).__init__(outcomes=['succeeded', 'failed', 'preempted'])
        
        self.GO_FORWARD_TIMEOUT = 100
        self.foundPathSub = rospy.Subscriber('img_rec/paths', ImgRecObject, self.found_path_cb)
        self.foundPathYet = False
        
    def execute(self, userdata):
        move('Depth', 'Command', 2.0)
        for i in range(4):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        forward(.75)
        for i in range(self.GO_FORWARD_TIMEOUT):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        return 'succeeded'
        
    def found_path_cb(self, msg):
    
        self.foundPathYet = True
