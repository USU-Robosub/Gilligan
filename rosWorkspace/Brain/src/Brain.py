#!/usr/bin/env python
import roslib; roslib.load_manifest('Brain')
import rospy
import smach
import smach_ros
import threading


from std_msgs.msg import UInt8
from Robosub.msg import HighLevelControl

from missions import QualifyPathMission as Mission
from utils import move


class Idle(smach_ros.MonitorState):
    def __init__(self):
        super(Idle, self).__init__('/Motor_State', UInt8, self.cb)
        self.shouldStartMission = False
    def execute(self, userdata):
        move('Depth', 'Command', 0)
        move('Forward', 'Command', 0)
        move('Turn', 'Manual', 0)
        return super(Idle, self).execute(userdata)
    def cb(self, userdata, msg):
        if msg.data == 1:
            if self.shouldStartMission:
                self.shouldStartMission = False  # Have to get this True again later
                return False  # Kick out and send 'invalid' outcome
            else:
                return True   # Keep going because we shouldn't start yet
        elif msg.data == 0:
            self.shouldStartMission = True
            return True


class Safety(smach.State):
    def __init__(self):
        super(Safety, self).__init__(outcomes=['preempted'])
    def execute(self, userdata):
        while not self.preempt_requested():
            rospy.sleep(1)
        self.service_preempt()
        return 'preempted'


class MissionMonitor(smach_ros.MonitorState):
    def __init__(self):
        super(MissionMonitor, self).__init__('/Motor_State', UInt8, self.cb)
    def cb(self, userdata, msg):
        return msg.data == 1


class SurfaceMonitor(smach.State):
    def __init__(self):
        super(SurfaceMonitor, self).__init__(outcomes=['preempted'])
    def execute(self, userdata):
        while not self.preempt_requested():
            rospy.sleep(1)
        self.service_preempt()
        return 'preempted'


if __name__ == '__main__':
    rospy.init_node('Brain')

    def child_term_cb(outcome_map):
        return ('Mission' in outcome_map and outcome_map['Mission'] == 'succeeded'
                or 'MissionMonitor' in outcome_map and outcome_map['MissionMonitor'] == 'invalid'
                or 'CompToggle' in outcome_map and outcome_map['CompToggle'] == 'preempted')

    sm_comp_run = smach.Concurrence(outcomes=['succeeded', 'failed', 'preempted'],
                                    default_outcome='preempted',
                                    outcome_map={'succeeded':{'Mission':'succeeded'},
                                                 'failed':{'Mission':'failed'}},
                                    child_termination_cb=child_term_cb)
    with sm_comp_run:
        smach.Concurrence.add('MissionMonitor', MissionMonitor())
        smach.Concurrence.add('SurfaceMonitor', SurfaceMonitor())
        smach.Concurrence.add('Mission', Mission())

    sm_comp_toggle = smach.StateMachine(outcomes=['preempted'],
                                        )
    with sm_comp_toggle:
        smach.StateMachine.add('Idle', Idle(), transitions={'invalid': 'CompRun',
                                                            'valid': 'CompRun'})
        smach.StateMachine.add('CompRun', sm_comp_run, transitions={'succeeded': 'Idle',
                                                                    'failed': 'Idle',
                                                                    'preempted': 'Idle'})

    sm_global = smach.Concurrence(outcomes=['preempted'],
                                  default_outcome='preempted',
                                  child_termination_cb=child_term_cb)
    with sm_global:
        smach.Concurrence.add('Safety', Safety())
        smach.Concurrence.add('CompToggle', sm_comp_toggle)

    sis = smach_ros.IntrospectionServer('smach_server', sm_global, '/sm_root')
    sis.start()
    sm_global.execute()
    rospy.spin()
    sis.stop()

