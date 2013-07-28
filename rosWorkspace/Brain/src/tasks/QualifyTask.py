import smach
import rospy
from utils import move
from SubImageRecognition.msg import ImgRecObject


class QualifyTask(smach.State):
    def __init__(self):
        super(QualifyTask, self).__init__(outcomes=['succeeded', 'failed', 'preempted'])
        
        self.DIVE_TIME = 4
        self.GO_FORWARD_TIMEOUT = 100
        self.foundPathSub = rospy.Subscriber('img_rec/paths', ImgRecObject, self.found_path_cb)
        self.foundPathYet = False

        
    def execute(self, userdata):
        move('Depth', 'Command', 0.0)
        for i in range(self.DIVE_TIME):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        move('Forward', 'Command', .25)
        for i in range(self.GO_FORWARD_TIMEOUT):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
            if self.foundPathYet:
                move('Forward', 'Command', 0.0)
                return 'succeeded'
        return 'succeeded'
        

    def found_path_cb(self, msg):
    
        self.foundPathYet = True