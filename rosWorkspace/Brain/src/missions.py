import smach
import rospy

import tasks

from utils import move




def QualifyMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('QualifyTask', tasks.QualifyTask())
    return sm
    
def QualifyPathMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('QualifyTask', tasks.QualifyTask(),
                                transitions={'succeeded':'PathTask'})
        smach.StateMachine.add('PathTask', tasks.PathTask())
    return sm