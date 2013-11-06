# Quick and dirty Brain tutorial #

## Getting Started ##

Smach is actually pretty striaghtforward, and if you're comfortable with python, it should be *relatively* clear what's happening with the Brain. 

### Read The Documentation ###

Give the [Smach Documentation][smach] at least a cursory glance before you attempt to go any further. Chris and I wrote the AI the night before the competition, so we really haven't built *that much* on top of what's there. If you understand what smach is doing, you'll understand what the Brain does. No magic here.

### Look at the visualizer! ###
![visualizee][visualize]

Smach comes with an incredibly handy state visualization tool called smach_viewer. The quickest way to grasp what the brain does is to open it up and poke around. 

On the sub: (or on a machine that has the sub configured as a ros master)

1. If it isn't already running, start roscore using the startupAuto.sh script
    If you're ssh'ed into the sub, make sure you are using the -X option
2. If nothing breaks, simply type 

        rosrun smach_viewer smach_viewer.py

3. Viola! This is the graph for the Brain. It's not that complicated right now, and if you just look at it for a bit, you should be able to get some intuition for all that is happening.

### States###

All you need to do to get started writing AI for gilligan is this boilerplate:

    import smach
    import rospy
    from utils import forward, dive #some high level movement utilities

    #All tasks are states or state machines
    class MyTask(smach.State): 
        def __init__(self):
            #Smach has to know what our possible outcomes are.
            #These could be anything, as long as the state machine(s) that
            #use this state is explicitly made aware of them
            super(MyTask, self).__init__(outcomes=['found_path', 'timed_out', 'preempted'])

        #Required. This is called when a state is entered.
        def execute(self):
            #If you need subs and pubs, do it here
            sub = rospy.Subscriber()
            pub = rospy.Publisher()
            
            #This bit of code ensures that the task doesn't block execution
            #if some higher state or state machine transitions and this state
            #is no longer needed.
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            #Do a bunch of very smart AI stuff...
            #These results must be in the list of declared outcomes.
            if some_condition:
                return 'some_result' #i.e. found_path or timed_out
            else:
                return 'some_other_result'

Alright, I know that's a lot of boilerplate, but it could be worse. With just that bit of code, you're now free to do any kind of crazy pythonic wizardry, and as long as you can condense your outcomes into a concise list of strings, (my favorites are succeeded, failed, and timed_out, but the sky is the limit), smach should be able to play nice.

### State Machines ###
State machines are just sets of states that have some high level cohesion such as a complicated task involving many states, or indeed an entire autonomous run. There is a lot of depth here, and a lot of power, so again I would reccommend that you go look at the [Smach Wiki][smach], but here's the quick and dirty. Here is an example state machine that we used to qualify in 2013:

    import tasks

    def QualifyPathMission():
        sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
        with sm:
            smach.StateMachine.add('QualifyTask', tasks.QualifyTask(),
                                    transitions={'found_path': 'PathTask',
                                    'timed_out': 'PathTask'})
            smach.StateMachine.add('PathTask', tasks.PathTask())
        return sm

I actually really like this setup, because I think it's really clear what is supposed to happen here just by reading it. 

First, we create a state machine, and specify the possible outcomes. Outcomes are like state outcomes for all intents and purposed, because state machines are nestable!

Second, we add every possible state/machine that should exist in the state machine, along with a dictionary of transitions, whose keys are outcomes of that state, and whose values are the name of the task that should be executed as a result of that outcome. 

To create a simple state machine, create your states in Brain/tasks, create a mission that uses those tasks in Brain/missions, and then change the import statement in Brain.py to:
    from tasks import [your task here] as Mission

and start up roscore and the Brain. Again, there's a lot of depth here, so read up on the [smach docs][smach]. Good luck!


[smach]: http://wiki.ros.org/smach "Smach Wiki"
[visualize]: img/visualize.png