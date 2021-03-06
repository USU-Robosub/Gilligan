#!/usr/bin/env python
import roslib; roslib.load_manifest('Brain')
import rospy
import smach
import smach_ros
from smach import Sequence

from std_msgs.msg import UInt8

class Foo(smach.State):
    def __init__(self, outcomes = ['success', 'aborted', 'preempted']):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'aborted'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo("Executing Foo")
        return 'success'
    
class Bar(smach.State):
    def __init__(self, outcomes = ['success', 'aborted', 'preempted']):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'aborted'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo("Executing Bar")
        return 'success'

class Bas(smach.State):
    def __init__(self, outcomes = ['success', 'aborted', 'preempted']):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['success', 'aborted'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo("Executing Bas")
        return 'success'
#initialize the state machine
def main():
    rospy.init_node("Brain")
    
    sq = Sequence(outcomes = ['success', 'aborted', 'preempted'],
#    sm = smach.StateMachine(outcomes=['outcome4'])
#    with sm:
#        smach.StateMachine.add('FOO', Foo(),
#                               transitions={'success':'BAR',
#                                            'aborted':'BAS'})
#        smach.StateMachine.add('BAR', Bar(),
#                               transitions={'success':'FOO',
#                                            'aborted':'BAS'})
#        smach.StateMachine.add('BAS', Bas(),
#                               transitions={'success':'BAS',
#                                            'aborted':'FOO'})
#        output = sm.execute()
        
    
    sq = Sequence(outcomes=['success', 'aborted', 'preempted'],
                       connector_outcome = 'success')
    with sq:
       Sequence.add('FOO', Foo())
       Sequence.add('BAR', Bar())
       Sequence.add('BAS', Bas())   
        
       
       output = sq.execute()
       
if __name__ == '__main__':
    main()
