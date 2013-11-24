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
                                transitions={'found_path': 'PathTask',
                                'timed_out': 'PathTask'})
        smach.StateMachine.add('PathTask', tasks.PathTask())
    return sm

def PathMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('PathTask', tasks.PathTask())
    return sm

def NewPathMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'timeout', 'preempted'])
    with sm:
        smach.StateMachine.add('NewPathTask', tasks.NewPathTask())
    return sm

def QualifyPathBuoyMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('QualifyTask', tasks.QualifyTask(),
                                transitions={'found_path':'PathTask', 'timed_out':'BuoyTask'})
        smach.StateMachine.add('PathTask', tasks.PathTask(),
                                transitions={'succeeded':'BuoyTask'})
        smach.StateMachine.add('BuoyTask', tasks.BuoyTask())
    return sm


def PracticeBuoyMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('BuoyTask', tasks.BuoyTask())
    return sm

def PracticeMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('QualifyTask', tasks.MoveToNextPath(),
                                transitions={'succeeded':'PathTask1'})
        smach.StateMachine.add('PathTask1', tasks.PathTask(),
                                transitions={'succeeded':'BuoyTask'})
        smach.StateMachine.add('MoveToNextPath', tasks.MoveToNextPath(),
                                transitions={'succeeded':'BuoyTask'})
        smach.StateMachine.add('BuoyTask', tasks.BuoyTask(),
                                transitions={'succeeded':'MoveToNextPath2'})
        smach.StateMachine.add('MoveToNextPath2', tasks.MoveToNextPath(),
                                transitions={'succeeded':'PathTask2'})
        smach.StateMachine.add('PathTask2', tasks.PathTask())
    return sm

def CompMission():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
    with sm:
        smach.StateMachine.add('QualifyTask', tasks.MoveToNextPath(),
                                transitions={'succeeded':'PathTask1'})
        smach.StateMachine.add('PathTask1', tasks.PathTask(),
                                transitions={'succeeded':'BuoyTask'})
        smach.StateMachine.add('MoveToNextPath', tasks.MoveToNextPath(),
                                transitions={'succeeded':'BuoyTask'})
        smach.StateMachine.add('BuoyTask', tasks.BuoyTask(),
                                transitions={'succeeded':'MoveToNextPath2'})
        smach.StateMachine.add('MoveToNextPath2', tasks.MoveToNextPath(),
                                transitions={'succeeded':'PathTask2'})
        smach.StateMachine.add('PathTask2', tasks.PathTask())
    return sm


