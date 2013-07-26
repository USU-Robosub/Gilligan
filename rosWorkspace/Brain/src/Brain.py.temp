#!/usr/bin/env python
import roslib; roslib.load_manifest('')
import rospy
import smach
import smach_ros
from std_msgs.msg import UInt8
#define state Idle
class Idle(smach.MonitorState):
    def __init__(self):
        super(Idle, self).__init__('/Motor_State', UInt8, self.cb)
        
        def cb(userdata, msg):
            if msg.data == 1:
                return True
            elif msg.data == 0:
                return False
                
            
        
#initialize the state machine
def main():
    rospy.init_node("Brain")
    
    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('Idle', Idle(), transitions={'invalid':})
        
                            