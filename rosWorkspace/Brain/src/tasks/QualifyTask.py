import smach
import rospy
from utils import forward, dive
from SubImageRecognition.msg import ImgRecObject

class QualifyTask(smach.State):
    def __init__(self, target_depth = 0.0, timeout = 50):
        super(QualifyTask, self).__init__(outcomes=['found_path', 'timed_out', 'preempted'])
        
        self._target_depth = target_depth
        self._dive_time = int(target_depth * 2)
        self._timeout = timeout
        self._slow_down_duration = 1
        
    def reset(self):
        self.found_path = False
        
    def slow_down_stop(self, event):
        self.reset()
    
    def execute(self, userdata):
        self.reset()
        sub = rospy.Subscriber('img_rec/paths', ImgRecObject, self.found_path_cb)
        dive(self._target_depth)
        for i in range(self._dive_time):
            if self.preempt_requested():
                self.service_preempt()
                sub.unregister()
                return 'preempted'
            rospy.sleep(1)
        forward(.3)
        for i in range(self._timeout):
            if self.found_path:
                sub.unregister()
                #Start slowing down
                forward(-.1)
                rospy.Timer(rospy.Duration(self._slow_down_duration), self.slow_down_stop, True)
                #done slowing down.
                sub.unregister()
                return 'found_path'
            rospy.sleep(1)
        sub.unregister()
        self.reset()
        return 'timed_out'

    def found_path_cb(self, msg):
    
        self.found_path = True