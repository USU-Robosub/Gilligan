#!/usr/bin/env python
import roslib 
roslib.load_manifest('SubStateMachine')
import rospy
import os
from time import time
import inspect
import glob
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from string import join
from Robosub.msg import ModuleEnableMsg
from Robosub.msg import HighLevelControl

currentClasses = set()
PublisherCache = {}
ListenerCache = {}
gTimers = []
epsilon = .1

PublisherCache["Module_Enable"] = rospy.Publisher("Module_Enable", ModuleEnableMsg);

class Timer:
	def __init__(self, Class, ms, effect):
		self.Class = Class
		self.endTime = time() + float(ms)/1000.0
		self.effect = effect

def checkTimer(timer):
	if timer.endTime < time():
		timer.effect.Execute(0);
		return False;
	return True;

class Effect:
	def __init__(self, etype, args):
		self.Type = etype
		self.Arguments = args

	def Execute(self, message):
		exeArgs = []
		for arg in self.Arguments:
			exeArgs.append(arg)
		if(self.Arguments[0] == "?MESSAGE"):
			exeArgs[0] = message.data
		if self.Type == 'transition':
			ChangeState(exeArgs[0])
		elif self.Type == 'message':
			SendMessage(exeArgs)
		else:
			print "Error: unknown action " + self.Type

class Trigger:
	def __init__(self, Class, topic, ttype, op, value, effect):
		self.mClassName = Class
		self.mTopic = topic
		self.mType = ttype
		self.mOp = op
		if(self.mType == 'int'):
			self.mValue = int(value)
		elif(self.mType == 'float'):
			self.mValue = float(value)
		else:
			self.mValue = value

		self.mEffect = effect

	def Check(self, message):
		if self.mOp == '=' and message.data == self.mValue:
			self.mEffect.Execute(message)
		elif self.mOp == '<' and message.data < self.mValue:
			self.mEffect.Execute(message)
		elif self.mOp == '>' and message.data > self.mValue:
			self.mEffect.Execute(message)
		elif self.mOp == '~' and message.data + epsilon > self.mValue and message.data - epsilon < self.mValue:
			self.mEffect.Execute(message)
		elif self.mOp == 'any':
			self.mEffect.Execute(message)
		else:
			print "no"

class TopicListener:
	def __init__(self, Topic, Type):
		if Type == 'int':
			self.Listener = rospy.Subscriber(Topic, Int32, self.callback)
		elif Type == 'float':
			self.Listener = rospy.Subscriber(Topic, Float32, self.callback)
		elif Type == 'string':
			self.Listener = rospy.Subscriber(Topic, String, self.callback)
		else:
			print "Error unsupported type for listener " + Type
		self.ActiveTriggers = [];

	def callback(self, data): 
		for trigger in self.ActiveTriggers:
			trigger.Check(data)

def getListener(Topic, Type):
	global ListenerCache
	if not(Topic in ListenerCache):
		ListenerCache[Topic] = TopicListener(Topic, Type)
	return ListenerCache[Topic]

def ChangeState(newState):
	global currentClasses
	if(openClassFile(newState) == 0):
		return
	print "Switching to state " + newState
	newClasses = getAllClasses(newState)
	for Class in currentClasses.difference(newClasses):
		Remove(Class)
	for Class in newClasses.difference(currentClasses):
		Add(Class)
	currentClasses = newClasses

def Add(Class):
	Setup(Class)
	AddTriggers(Class)
	AddTimers(Class)

def Remove(Class):
	TearDown(Class)
	RemoveTriggers(Class)
	RemoveTimers(Class)

def Setup(Class):
	print "Seting up " + Class
	File = openClassFile(Class)
	sections = File.split('<')
	for section in sections:
		temp = section.split('>')
		if(len(temp) == 2):
			Type = temp[0]
			Data = temp[1]
			if Type == "Setup":
				DispatchMessages(Data)

def TearDown(Class):
	print "Tearing Down " + Class
	File = openClassFile(Class)
	sections = File.split('<')
	for section in sections:
		temp = section.split('>')
		if(len(temp) == 2):
			Type = temp[0]
			Data = temp[1]
			if Type == "TearDown":
				DispatchMessages(Data)

def openClassFile(Class):
	path = os.path.dirname( inspect.getfile(inspect.currentframe()) )
	path = path + '/../class'
	fileSys = os.walk(path)
	for root, subFolders, Files in fileSys:
		for File in Files:
			if File.endswith(".class"):
				Name = File[:-6]
				if Name == Class:
					return open(root + '/' + File).read()
	print "Could not open class \"" + Class + "\""
	return 0


def getPublisher(Class, Type):
	global PublisherCache
	try:
		if not Class in PublisherCache:
			PublisherCache[Class] = rospy.Publisher(Class, Type)
			rospy.sleep(1)
		return PublisherCache[Class]
	except:
		pass

def SendMessage(fields):
	if fields[1] == 'string':
		pub = getPublisher(fields[0], String)
		pub.publish(String(join(fields[2:])))
	elif fields[1] == 'float':
		pub = getPublisher(fields[0], Float32)
		pub.publish(Float32(fields[2]))
	elif fields[1] == 'int':
		pub = getPublisher(fields[0], Int32)
		pub.publish(Int32(fields[2]))
	elif fields[1] == 'activate':
		pub = getPublisher(fields[0], ModuleEnableMsg)
		pub.publish(ModuleEnableMsg(Module=fields[2], State=True));
	elif fields[1] == 'deactivate':
		pub = getPublisher(fields[0], ModuleEnableMsg)
		pub.publish(ModuleEnableMsg(Module=fields[2], State=False));
	elif fields[1] == 'Move':
		pub = getPublisher(fields[0], HighLevelControl)
		pub.publish(HighLevelControl(Direction=fields[2], MotionType="Offset", Value=float(fields[3])))

def DispatchMessages(Data):
	Data = Data.strip()
	messages = Data.split('\n')
	for message in messages:
		fields = message.split()
		if fields[0] == 'message':
			SendMessage(fields[1:])


def getAllClasses(Class):
	return getAllClassesHelper(Class, set())

def getAllClassesHelper(Class, used):
	used = used.union([Class])
	File = openClassFile(Class)
	sections = File.split('<')
	for section in sections:
		temp = section.split('>')
		if(len(temp) == 2):
			Type = temp[0]
			Data = temp[1]
			if Type == "Inherits":
				Data = Data.split()
				for thing in Data:
					if thing in used:
						pass
					else:
						used = getAllClassesHelper(thing, used)
				return used
	return used


#Timer Format:
#	After <ms> milliseconds do <action> <arg1> ...

def addTimer(Class, timer):
	global gTimer;
	if timer.strip() == '':
		return
	fields = timer.split()
	if(len(fields) < 6):
		print "error parsing timer too few arguments"
		print Class + "->" + timer
		return
	if fields[0].lower() != 'after' or fields[3] != 'do':
		print "error parsing timer"
		print Class + "->" + timer
		return
	if fields[2] == 'milliseconds' or fields[2] == 'ms':
	 	milliseconds = float(fields[1]);
	if fields[2] == 'seconds' or fields[2] == 's':
		milliseconds = float(fields[1]) * 1000;
	elif fields[2] == 'minutes' or fields[2] == 'm':
		milliseconds = float(fields[1]) * 60000;
	elif fields[2] == 'hours' or fields[2] == 'h':
		milliseconds = float(fields[1]) * 3600000;
	elif fields[2] == 'days' or fields[2] == 'd':
		milliseconds = float(fields[1]) * 86400000;
	else:
		print "error parsing timer, unknown time units"
		print Class + "->" + timer
		return
		
	EffectType = fields[4]
	EffectArgs = fields[5:]
	timer = Timer(Class, milliseconds, Effect(EffectType, EffectArgs))
	gTimers.append(timer)

def AddTimers(Class):
	File = openClassFile(Class)
	sections = File.split('<')
	for section in sections:
		temp = section.split('>')
		if(len(temp) == 2):
			Type = temp[0]
			Data = temp[1]
			if Type == "Timers":
				for timer in Data.split('\n'):
					if timer.strip() == '':
						continue
					addTimer(Class, timer)

def RemoveTimers(Class):
	global gTimers;
	gTimers = filter(lambda x: x.Class != Class, gTimers);
				
#Trigger Format:
#   <Channel> <Message_Type> <op> <value> triggers <action> <arg1> ...
# Supported Message_Types:
#   Int - std_msgs/Int32
#   Float - std_msgs/Float32
#   ImageProcMessage - whatever this is...
#
# Supported operations:
#   <
#   >
#   = (equality)
#   ~ (approximate equality for floats)
#   any (any message will execute the trigger)
#
# Supported Actions:
#   transition <State>
#   message <Channel> <Type> <Values>
#

def AddTrigger(Class, trigger):
	if trigger.strip() == '':
		return
	fields = trigger.split()
	if(len(fields) < 7):
		print "error parsing trigger too few arguments"
		print trigger
		return
	Topic = fields[0]
	Type = fields[1]
	Op = fields[2]
	Value = fields[3]
	if fields[4] != 'triggers':
		print "error parsing trigger"
		print Class + "->" + trigger
		return
	EffectType = fields[5]
	EffectArgs = fields[6:]
	trigger = Trigger(Class, Topic, Type, Op, Value, Effect(EffectType, EffectArgs))
	Listener = getListener(Topic, Type)
	Listener.ActiveTriggers.append(trigger)


def AddTriggers(Class):
	File = openClassFile(Class)
	sections = File.split('<')
	for section in sections:
		temp = section.split('>')
		if(len(temp) == 2):
			Type = temp[0]
			Data = temp[1]
			if Type == "Triggers":
				Triggers = Data.split('\n')
				for trigger in Triggers:
					if(trigger.strip() == ''):
						continue
					AddTrigger(Class, trigger)

def RemoveTriggers(Class):
	global ListenerCache
	for topic in ListenerCache:
		count_Triggers = len(ListenerCache[topic].ActiveTriggers)
		ListenerCache[topic].ActiveTriggers = filter( lambda trigger: trigger.mClassName != Class, ListenerCache[topic].ActiveTriggers)
		removed_Triggers = count_Triggers - len(ListenerCache[topic].ActiveTriggers)

def checkTimers():
	global gTimers;
	gTimers = filter(checkTimer, gTimers);

def main():
	rospy.init_node('StateMachine')
	ChangeState("StateChanger")
	rate = rospy.Rate(20);
	try:
		while not rospy.is_shutdown():
			checkTimers();
			rate.sleep();
	except KeyboardInterrupt:
		pass

main()
