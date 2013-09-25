import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, String
import time

from Robosub.msg import ModuleEnableMsg
from SubImageRecognition.msg import ImgRecObject

from utils import dive, turn, forward, strafe, ScanNarrow, move	


class GotoBuoyDepth(smach_ros.MonitorState):
    def __init__(self, target=0.0, threshold=0.5):
        super(GotoBuoyDepth, self).__init__('Sub_Depth', Float32, self.depth_cb)
        self._target = target
        self._threshold = threshold
    def execute(self, userdata):
        move('Depth', move.COMMAND, self._target)
        return super(GotoBuoyDepth, self).execute(userdata)
    def depth_cb(self, userdata, msg):
        return abs(msg.data - self._target) > self._threshold


class BuoyVisibleCheck(smach.State):
    def __init__(self, timeout=2):
        super(BuoyVisibleCheck, self).__init__(outcomes=['visible', 'not_visible', 'preempted'])
        self._timeout = timeout
    def execute(self, userdata):
        self.obj_found = False
        sub = rospy.Subscriber('img_rec/buoys/red', ImgRecObject, self.obj_cb)
        for i in range(self._timeout):
            if self.obj_found:
                sub.unregister()
                return 'visible'
            if self.preempt_requested():
                sub.unregister()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        sub.unregister()
        return 'not_visible'
    def obj_cb(self, msg):
        self.obj_found = True


class CenterOnBuoy(smach.State):
    def __init__(self, timeout=2, gain_x=-0.01, gain_y=0.01):
        super(CenterOnBuoy, self).__init__(outcomes=['succeeded', 'lost_buoy', 'preempted'])
        self._timeout = timeout
        self._gain_x = gain_x
        self._gain_y = gain_y

    def execute(self, userdata):
        self.reset()
        self.buoy_sub = rospy.Subscriber('img_rec/buoys/red', ImgRecObject, self.buoy_cb)
        self.depth_sub = rospy.Subscriber('Sub_Depth', Float32, self.depth_cb)

        while True:
            if self.preempt_requested():
                self.service_preempt()
                self.unsub()
                return 'preempted'
            delta = time.time() - self.last_time
            if self.last_buoy and delta < self._timeout:
                if abs(self.last_buoy.center_x) < 50:
                    self.good_x_count += 1
                    strafe(self.last_buoy.center_x * self._gain_x)
                else:
                    self.good_x_count = 0
                    strafe(0)
                if abs(self.last_buoy.center_y) < 50:
                    self.good_y_count += 1
                    dive(self.last_buoy.center_y * self._gain_y)
                else:
                    self.good_y_count = 0
                    if self.last_depth:
                        dive(self.last_depth - self.last_buoy.center_y * self._gain_y)
                if self.good_x_count >= 10 and self.good_y_count >= 10:
                    self.good_y_count += 1
                    if self.last_depth:
                        dive(self.last_depth)
                self.last_buoy = None
            elif delta > self._timeout:
                self.unsub()
                return 'lost_buoy'
            rospy.sleep(0.5)
        self.buoy_sub.unregister()
        self.depth_sub.unregister()
        return 'succeeded'

    def buoy_cb(self, msg):
        self.last_buoy = msg
        self.last_time = time.time()

    def depth_cb(self, msg):
        self.last_depth = msg.data

    def unsub(self):
        self.buoy_sub.unregister()
        self.depth_sub.unregister()
        self.reset()

    def reset(self):
        self.last_buoy = None
        self.last_time = time.time()
        self.last_depth = None
        self.good_x_count = 0
        self.good_y_count = 0


class GoForwardOnCenter(smach.State):

    RES_W = 768

    def __init__(self, close_w = 800, close_h = 600, timeout = 2, threshold_gains = 300):
        super(GoForwardOnCenter, self).__init__(outcomes=['close_enough', 'lost_buoy', 'alignment_needed', 'preempted'])
        self._close_w = close_w
        self._close_h = close_h
        self._timeout = timeout
        self._threshold_gains = threshold_gains
        
    def reset(self):
        self.alignment_needed = False
        self.is_close_enough = False
        self.buoy_lost_timer = self._timeout
        
    def execute(userdata, self):
        self.reset()
        sub = rospy.Subscriber('img_rec/buoys/red', ImgRecObject, self.check_progress_cb)
        
        while not self.is_close_enough:
            for i in range(self._timeout):
                if self._lost_buoy_timer <= 0:
                    #Too much time has passed without seeing the buoy
                    sub.unregister()
                    self.reset()
                    return 'lost_buoy' 
                if self.alignment_needed:
                    #Too far from center
                    sub.unregister()
                    self.reset()
                    return 'alignment_needed'
                if self.preempt_requested():
                    sub.unregister()
                    self.service_preempt()
                    self.reset()
                    return 'preempted'
                rospy.sleep(1)
                self._lost_buoy_timer -= 1
                
        #We're probably close enough
        sub.unregister()
        self.reset()
        return 'close_enough'
            
    def check_progress_cb(self, msg):
        self._lost.buoy.timer = self._timeout
        threshold = self._threshold_gains * msg.width/RES_W
        
        if msg.width > self._close_w or msg.height > self._close_h:
            self.is_close_enough = True
        elif (msg.center_x > threshold or msg.center_x < -threshold 
                or msg.center_y > threshold or msg.center_y < -threshold):
            self.alignment_needed = True


class BumpBuoy(smach.State):
    def __init__(self):
        super(BumpBuoy, self).__init__(outcomes=['succeeded', 'preempted'])
    def execute(self, userdata):
        forward(0.5)
        for i in range(1000):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1)
        return 'preempted'


def BuoyTask():
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted'])
    with sm:
        smach.StateMachine.add('GotoBuoyDepth', GotoBuoyDepth(),
                               transitions={'valid':'preempted',
                                             'invalid':'ScanNarrow',
                                             'preempted':'preempted'})
        smach.StateMachine.add('BuoyVisibleCheck', BuoyVisibleCheck(),
                               transitions={'visible': 'CenterOnBuoy',
                                             'not_visible': 'ScanNarrow',
                                             'preempted': 'preempted'})
        smach.StateMachine.add('ScanNarrow', ScanNarrow('img_rec/buoys/red'),
                               transitions={'succeeded': 'CenterOnBuoy', 'timed_out': 'ScanWide'})
        smach.StateMachine.add('ScanWide', ScanNarrow('img_rec/buoys/red', 180, 0, scan_duration=1000),
                               transitions={'succeeded': 'CenterOnBuoy',
                                            'timed_out': 'preempted',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('CenterOnBuoy', CenterOnBuoy(),
                               transitions={'succeeded': 'GoForwardOnCenter',
                                            'lost_buoy': 'ScanNarrow',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('GoForwardOnCenter', GoForwardOnCenter(),
                               transitions={'close_enough': 'BumpBuoy',
                                            'lost_buoy': 'ScanNarrow',
                                            'alignment_needed': 'CenterOnBuoy',
                                            'preempted': 'preempted'})
        smach.StateMachine.add('BumpBuoy', BumpBuoy(),
                               transitions={'succeeded': 'succeeded',
                                            'preempted': 'preempted'})
    return sm


#class BuoyTask(smach.State):
#    def __init__(self):
#        super(BuoyTask, self).__init__(outcomes=['succeeded', 'preempted'])
#        self.is_complete = False
#        self.buoy_seen = False
#        self.enabled = False
#        self.BUOY_DEPTH = 7.0
#        
#        #Subscribe and publish
#        self.pub = rospy.Publisher('/Module_Enable', ModuleEnableMsg)
#        self.sub = rospy.Subscriber('/Task_Completion', String, self.task_complete)
#    def execute(self, userdata):
#        move('Depth', 'Command', self.BUOY_DEPTH)
#        
#        #Begin looking for a red buoy
#        self.found_buoy_sub = rospy.Subscriber('/img_rec/buoys/red', ImgRecObject, self.enable_task_cb)
#        for i in range(100):
#            if self.is_complete:
#                rospy.loginfo('complete!')
#                self.disable_task()
#                return 'succeeded'
#            if self.preempt_requested():
#                self.disable_task()
#                self.service_preempt()
#                return 'preempted'
#            rospy.sleep(1)
#        self.disable_task()
#        rospy.loginfo("foo")
#        return 'succeeded'
#    def disable_task(self):
#        msg = ModuleEnableMsg()
#        msg.Module = 'BuoyTask'
#        msg.State = False
#        self.enabled = False
#        self.pub.publish(msg)
#    def enable_task_cb(self, msg):
#        if not self.enabled:
#            msg = ModuleEnableMsg()
#            msg.Module = 'BuoyTask'
#            msg.State = True
#            self.enabled = True
#            rospy.loginfo("Sending Module_Enable message: %s, %i", msg.Module, msg.State)
#            self.pub.publish(msg)
#    
#    def task_complete(self, msg):
#        self.is_complete = True
