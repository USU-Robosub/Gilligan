import smach
import rospy
from utils import move

class ReverseTask(smach.State):
    def __init__(self):
        super(ReverseTask, self).__init__(outcomes=['succeeded', 'failed', 'preempted'])
        
        self.DIVE_TIME = 4
        self.GO_BACKWARD_TIMEOUT = 40
        self.TIME_SLICE = 1

        
    def execute(self, userdata):
        move('Depth', 'Command', 3.0)
        for i in range(self.DIVE_TIME):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(self.TIME_SLICE)
        move('Forward', 'Command', -.75)
        for i in range(4):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(self.TIME_SLICE)
        return 'succeeded'         