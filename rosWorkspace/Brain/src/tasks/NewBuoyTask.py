import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, String
import time

from Robosub.msg import ModuleEnableMsg
from SubImageRecognition.msg import ImgRecObject

from utils import dive, turn, forward, strafe, ScanNarrow, move	

class NewBuoyTask(smach.State):
    def __init__(self):
        super(NewBuoyTask, self)._init__(outcomes=['succeeded', 'preempted', 'timeout'])
        self.buoyHit = False
        self.timeout = 100
    def extendTimeout(self):
        if(self.timeout < 30):
            self.timeout = 30
    def execute(self, userdata):
        self.publisher = rospy.Publisher('/Module_Enable', ModuleEnableMsg)
        self.subscriber = rospy.Subscriber('/Task_Completion', String, self.taskCompleted)
        self.buoySubscriber = rospy.Subscriber('img_rec/buoys/red', ImgRecObject, self.buoyLoc)
        self.depthSubscriber = rospy.Subscriber('Sub_Depth', Float32, self.depth)
        msg = ModuleEnableMsg()
        msg.Module = 'NewBuoyTask'
        msg.State = True
        self.publisher.publish(msg)
        #keep trying until preempted, success, or timeout
        while self.timeout > 0:
            if self.buoyHit:
                self.beDone()
                return 'succeeded'
            if self.preempt_requested():
                self.beDone()
                self.service_preempt()
                return 'preempted'
            if self.objectLost:
                self.scanForBuoy()
            else:
                self.descendToBuoy()
                self.alignWithBouy()
                self.advance()
                self.extendTimeout()
            # we decide the object is lost until we receive another message
            self.objectLost = True
            rospy.sleep(1)
            self.timeout -= 1
            self.objectLost = True
        #we timed out
        self.beDone()
        return 'timeout'
    def taskCompleted(self):
        if msg.data == 'NewBuoyTask':
            self.buoyHit=True
    def beDone(self):
        msg = ModuleEnableMsg()
        msg.Module = 'NewBuoyTask'
        msg.State = False
        self.publisher.publish(msg)
    def descendToBuoy(self):
        #dive to buoy depth
        if(self.lastKnownBuoyLoc.center_y > 50):
            dive(-.1)
        elif(self.lastKnownBuoyLoc.center_y < -50):
            dive(.1)
        else:
            dive(0)

    def scanForBuoy(self):
        # we don't know where it is so let's try right
        # we'll go into a circle til we time out or find the buoy
        dive(0)
        strafe(0)
        turn(.1)


    def alignWithBouy(self):
        #align x
        if(self.lastKnownBuoyLoc.center_x > 50):
            strafe(-.1)
        elif(self.lastKnownBuoyLoc.center_x < -50):
            strafe(.1)
        else:
            strafe(0)

    def advance(self):
        #if it's taking up the whole camera it's definately hit or just about to
        #not sure what the whole camera is though.
        if(self.lastKnownBuoyLoc.width < 500):
            forward(.3)
        else:
            forward(0.0)

    def buoyLoc(self, msg):
        self.objectLost = False
        self.lastKnownBuoyLoc = msg
        self.prevTime = time.time()
    def depth(self, msg):
        self.lastKnownDepth = msg.data



