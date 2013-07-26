#!/usr/bin/env python
import roslib; roslib.load_manifest('')
import rospy
import smach
import smach_ros
from smach import Sequence

import Idle

from std_msgs.msg import UInt8

class Foo(smach.State):
    def __init__(self, outcomes = ['success', 'aborted', 'preempted']):
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo("Executing Foo")
        return 'success'
    
class Bar(smach.State):
    def __init__(self, outcomes = ['success', 'aborted', 'preempted']):
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo("Executing Bar")
        return 'success'

class Bas(smach.State):
    def __init__(self, outcomes = ['success', 'aborted', 'preempted']):
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo("Executing Bas")
        return 'success'
#initialize the state machine
def main():
    rospy.init_node("Brain")
    
    sq = Sequence(outcomes = ['success', 'aborted', 'preempted'],
                       connector_outcome = 'success')
    with sq:
       Sequence.add('FOO', Foo())
       Sequence.add('BAR', Bar())
       Sequence.add('BAS', Bas())   
        
                            