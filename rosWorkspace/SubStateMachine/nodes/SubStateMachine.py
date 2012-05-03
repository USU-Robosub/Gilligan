#!/usr/bin/env python
import roslib 
roslib.load_manifest('SubStateMachine')
import rospy
import os
import inspect
import glob
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from string import join

currentClasses = set()
PublisherCache = {}
ListenerCache = {}
epsilon = .1

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

def Remove(Class):
	TearDown(Class)
	RemoveTriggers(Class)

def Setup(Class):
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
		PublisherCache[Class] = rospy.Publisher(Class, Type)
		rospy.sleep(.1)
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
		pub = rospy.Publisher(fields[0], Int32)
		pub.publish(Int32(fields[2]))

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

def main():
	rospy.init_node('StateMachine')
	ChangeState("StateChanger")
	try:
		rospy.spin()
	except KeyboardInterrupt:
		pass

main()
