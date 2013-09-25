import rospy
import smach

from Robosub.msg import HighLevelControl
from SubImageRecognition.msg import ImgRecObject


def move(direction, motion_type, value):
    move.msg.Direction = direction
    move.msg.MotionType = motion_type
    move.msg.Value = value
    move.pub.publish(move.msg)
move.msg = HighLevelControl()
move.pub = rospy.Publisher('/High_Level_Motion', HighLevelControl)
move.COMMAND = 'Command'
move.FORWARD = 'Forward'
move.MANUAL = 'Manual'
move.STRAFE = 'Strafe'
move.TURN = 'Turn'
move.DIVE = 'Depth'


def turn(value):
    move(move.TURN, move.COMMAND, value)


def forward(value):
    move(move.TURN, move.COMMAND, 0)
    move(move.FORWARD, move.COMMAND, value)


def strafe(value):
    move(move.STRAFE, move.COMMAND, value)
    
def dive(value):
    move(move.DIVE, move.COMMAND, value)
    

class ScanNarrow(smach.State):
   
    
    def __init__(self, img_rec_topic, scan_degrees_start = 15, reverse_speed = -.5, scan_gains = 1, scan_duration = 1):
    
        super(ScanNarrow, self).__init__(outcomes=['succeeded', 'timed_out', 'preempted']) 
        self._img_rec_topic = img_rec_topic
        self._scan_degrees_start = scan_degrees_start * scan_gains
        self._reverse_gains = reverse_speed
        self._reverse_duration = reverse_speed * -4
        self._scan_duration = scan_duration
        self._reverse_speed = reverse_speed
        
        
    def reset(self):
        self.scan_degrees = self._scan_degrees_start
        self.obj_found = 0
        self.scan_count = 0
        
    def execute(self, userdata):
        self.reset()
        sub = rospy.Subscriber(self._img_rec_topic, ImgRecObject, self.obj_found_cb)
        
        while not self.obj_found:
            #Back up a little bit
            forward(self._reverse_speed)
            rospy.sleep(self._reverse_duration)
            forward(0.0)
            
            #Scan to the left
            turn(-self.scan_degrees)
            for i in range(self._scan_duration):
                if self.preempt_requested():
                    self.service_preempt()
                    self.reset()
                    return 'preempted'
                rospy.sleep(1)
            
            #Scan to the right
            turn(self.scan_degrees * 2)
            for i in range(self._scan_duration * 2):
                if self.preempt_requested():
                    self.service_preempt()
                    self.reset()
                    return 'preempted'
                rospy.sleep(1)
            
            #return to center (probably)
            turn(-self.scan_degrees)
            for i in range(self._scan_duration):
                if self.preempt_requested():
                    self.service_preempt()
                    self.reset()
                    return 'preempted'
                rospy.sleep(1)
            
            self.scan_count += 1
            
        self.reset()
        return 'succeeded'
    
    def obj_found_cb(self, msg):
        self.obj_found = True

## I started working on this to add a timeout but gave up for now  -Chris
#class MonitorState(smach.State):
#    def __init__(self, topic, msg_type, cond_cb, max_checks=-1):
#        smach.State.__init__(self,outcomes=['valid','invalid','preempted'])

#        self._topic = topic
#        self._msg_type = msg_type
#        self._cond_cb = cond_cb
#        self._max_checks = max_checks
#        self._n_checks = 0

#        self._trigger_cond = threading.Condition()

#    def execute(self, ud):
#        self._n_checks = 0

#        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb, callback_args=[ud])

#        with self._trigger_cond:
#            self._trigger_cond.wait()

#        self._sub.unregister()

#        if self.preempt_requested():
#            self.service_preempt()
#            return 'preempted'

#        if self._max_checks > 0 and self._n_checks >= self._max_checks:
#            return 'valid'

#        return 'invalid'

#    def _cb(self, msg, ud):
#        self._n_checks += 1
#        try:
#            if (self._max_checks > 0 and self._n_checks >= self._max_checks) or not self._cond_cb(ud, msg):
#                self._wake()
#        except:
#            rospy.logerr("Error thrown while executing condition callback %s" % str(self._cond_cb))
#            self._wake()

#    def request_preempt(self):
#        smach.State.request_preempt(self)
#        self._wake()

#    def _timeout_cb(self):
#        self._wake()

#    def _wake(self):
#        with self._trigger_cond:
#        self._trigger_cond.notify()

