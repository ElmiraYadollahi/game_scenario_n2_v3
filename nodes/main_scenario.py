#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import animations.embarassed_seated_pose
import animations.scratchHead_seated_pose
import animations.IdontKnow_seated_pose
import animations.IdontKnow2_seated_pose
import animations.hesitation_seated_pose
import animations.hesitation2_seated_pose
import animations.hesitation3_seated_pose
import animations.thinking5_seated_pose
import animations.thinking6_seated_pose
import animations.thinking7_seated_pose
import animations.thinking8_seated_pose
import animations.monster_seated_pose
import animations.disappointed_seated_pose
import animations.excited_seated_pose
import animations.excited2_seated_pose
import animations.happy_seated_pose
import animations.happy2_seated_pose
import animations.happy3_seated_pose
import animations.introduction_pose
import animations.proud_seated_pose
import animations.nod_pose
import animations.relieved_seated_pose
import animations.winner_seated_pose
import animations.winner2_seated_pose
import animations.pensive_seated_pose


# Idle Movement
import animations.lookHand_seated_pose
import animations.relaxation_seated_pose
import animations.scratchHand_seated_pose
import animations.scratchHead_seated_pose


from naoqi import ALProxy 
from naoqi import ALModule
from naoqi import ALBroker
import codecs
import time
import re
import random
import threading 

# Debug
import rospy
#from memory.msg import Animation

import sys
import motion
import almath
import math
from std_msgs.msg import Int64, Int64MultiArray, String, Bool, Float32
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
from numpy.linalg import inv
from EEClass import EYEEMOTIONS
from MTClass import TRANSFORMATION
from WPClass import WORDPROCESSING
from RTClass import RECORDANDTRANSCRIBE
from IMClass import IDLEMOVEMENTTHREADING
from MAClass import MOTIONANIMATIONSELECTION
from logger import logger



# GLOBALS
######################################################
global counter
counter = 0

global tag_pairs
tag_pairs = '[0, 0]'

global moveHand 
moveHand = 0

global trajComplete
trajComplete = []

global correctPermission
correctPermission = False

global cardPosition
cardPosition = 0

global reactionPermission 
reactionPermission = False

global FinishSign
FinishSign = False

global lastBreakPoint 
lastBreakPoint = [0, 0]

global effector
effector = "RArm"

global once
once = 0

global finishProTask 
finishProTask = False

global finishCompTask 
finishCompTask = False

global prodSeries

global RobotConnected
RobotConnected = True

global yekan
yekan = 0

global dahgan 
dahgan = 0

global sadgan
sadgan = 0

global moveHand
moveHand = 0

global if_wait
if_wait = False

initialization_dict ={	'ChildName' : "None",
						'MoveHandSpeed'	: 0.1,
						'ReadDelay'	: 0.1,
						'ReadSpeed'	: "70",
						'ReactionDelay' : 1,
						'HandPoint' : False
					 }
					 
working_dict = {}

confusion_matrix_dict = { 'equation'	: "condition"
						
						}

compType_dict = {	'ambiguous'		: 1,
					'egocentric'	: 2,
					'addressee'		: 3,
				}

robot_answers_dict = {	1 : True,
						2 : True,
						3 : True,
						4 : True,
						5 : True 
					}

row_count_dict = {	'yekan' : 0,
						'dahgan' : 0,
						'sadgan' : 0,
					}

robot_move_dict = {
	
}

"""basic_data_dict = { 'length': 1500,
					'radius': 100
}"""

basic_data_dict = { 'length': 737,
					'radius': 50
}

button_dict = { 'correct': "No_signal",
				'wrong': "No_signal",
				'reset': "No_signal"
}

class MOTION_ANIMATION_SELECTION:
	def __init__(self):
		n = 1


	def reactionToREDCard(self):
		""" Select a motionProxy from the available motionProxys after receiving 

		"""
		
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		motionProxyNum = random.randint(1,5)
		#motionProxyNum = 4

		if motionProxyNum == 6:
			wordsBefore = "\\rspd=90\\ Oh!! I was not careful!!"
			sleepTime = 1
			wordsAfter = "\\rspd=90\\wait!! \\pau=500\\ I'll try again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.embarassed_seated_pose, wordsBefore, wordsAfter, sleepTime)

		if motionProxyNum == 2:
			wordsBefore = "\\rspd=60\\ Aaahhh!! \\pau=600\\ \\rspd=90\\ I made a mistake!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ let me try again"
			emotion = "surprise"
			reactToTheMistake(emotion, animations.scratchHead_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 7:
			wordsBefore = "\\rspd=80\\ Oh!! I was not careful!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=90\\ I will read it again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.disappointed_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 4:
			wordsBefore = "\\rspd=70\\ Oh!! sorry!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ I will read it again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.pensive_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

		if motionProxyNum == 5:
			wordsBefore = "\\rspd=70\\ hmm!! \\pau=700\\ I was wrong"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ I need to read it again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.thinking6_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 1:
			wordsBefore = "\\rspd=80\\ oh  \\pau=700\\ \\rspd=60\\  really?!!"		
			sleepTime = 1
			wordsAfter = " \\rspd=80\\ \\pau=200\\ let me read it again"
			emotion = "surprise"
			reactToTheMistake(emotion, animations.hesitation2_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 3:
			wordsBefore = "\\rspd=70\\ Oh!!! \\rspd=80\\ \\pau=700\\ I was wrong"		
			sleepTime = 1
			wordsAfter = "\\rspd=90\\ I will try again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.thinking5_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)


	def reactionToGREENCard(self):
		""" Select a motionProxy from the available motionProxys after receiving the green card 

		"""
		
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		motionProxyNum = random.randint(1,5)
		#motionProxyNum = 4
		emotion = "happy"

		if motionProxyNum == 1:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = " \\rspd=80\\  awesome!!! Thank you"
			sleepTime = 0.5
			wordsAfter = "\\rspd=70\\ \\style=neutral\\"
			reactToTheMistake(emotion, animations.winner_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 2:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = " \\rspd=60\\  Yeaaah!!! this is great"		
			sleepTime = 0.5
			wordsAfter = "\\rspd=80\\ \\style=neutral\\"
			reactToTheMistake(emotion, animations.winner2_seated_pose, wordsBefore, wordsAfter, sleepTime, 1.0)

		if motionProxyNum == 3:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = " \\rspd=80\\  Yeaaah!!! I'm getting better"		
			sleepTime = 0.5
			wordsAfter = "\\rspd=80\\  \\style=neutral\\"
			reactToTheMistake2(animations.relieved_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 4:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = " \\rspd=80\\  Wow!!!  I was right this time"		
			sleepTime = 0.5
			wordsAfter = "\\rspd=80\\ \\style=neutral\\"
			reactToTheMistake(emotion, animations.proud_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

		if motionProxyNum == 5:
			wordsBefore = " \\rspd=80\\  Yeaaah!!! I read it correctly"		
			sleepTime = 0.5
			wordsAfter = "\\rspd=80\\ \\style=neutral\\"
			reactToTheMistake(emotion, animations.happy_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 6:
			wordsBefore = " \\rspd=70\\ I'm so happy,  I was right!!!"		
			sleepTime = 0.5
			wordsAfter = "\\rspd=70\\ \\style=neutral\\"
			reactToTheMistake(emotion, animations.happy2_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)

		if motionProxyNum == 7:
			wordsBefore = " \\rspd=70\\ Yeaaah!!! I'm improving"		
			sleepTime = 0.5
			wordsAfter = " \\rspd=80\\  this is great \\style=neutral\\"
			reactToTheMistake(emotion, animations.happy3_pose, wordsBefore, wordsAfter, sleepTime)


	def reactionIdleMovement(self):
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		motionProxyNum = random.randint(1,4)
		#motionProxyNum = 4
		emotion = "happy"

		if motionProxyNum == 1:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ hmm"
			sleepTime = 3
			reactToBoredness(emotion, animations.scratchHead_seated_pose, sleepTime, wordsAfter, 0.7)

		if motionProxyNum == 2:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ hmm, I'm sleepy"			
			sleepTime = 2
			reactToBoredness(emotion, animations.scratchHand_seated_pose, sleepTime, wordsAfter, 0.7)

		if motionProxyNum == 3:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ hmm, I'm tired"
			sleepTime = 2
			reactToBoredness(emotion, animations.lookHand_seated_pose, sleepTime, wordsAfter, 0.7)

		if motionProxyNum == 4:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ I'm so \\rspd=50\\ bored, \\rspd=80\\ \\pau=50\\ you don't want to read with me?"
			sleepTime = 2
			reactToBoredness(emotion, animations.relaxation_seated_pose, sleepTime, wordsAfter, 0.7)


	def reactionToSpecificTags(self):
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		
		wordsAfter = ""
		sleepTime = 2
		emotion = "anger"
		reactWithSpecificPoses(emotion, animations.monster_seated_pose, sleepTime, wordsAfter, 0.7)



def print_row(data):
	print ("-------------------------------------------------")
	print (data)


def TrialBehaviour():
	TrialNumber = 0
	Response = True
	print robot_answers_dict
	
	TrialNumber += 1
	order = random.sample(range(2,5), 2)
	ordered_order = sorted(order, key=int)
	robot_answers_dict[order[0]] = False
	robot_answers_dict[order[1]] = False

def PrintRadius(data):
	button_dict['correct'] = data.data

def PrintRadius(data):
	button_dict['wrong'] = data.data

def PrintRadius(data):
	button_dict['reset'] = data.data

def PrintLength(data):
	basic_data_dict['length'] = int(float(data.data)) + 1 - basic_data_dict['radius']/2

def PrintRadius(data):
	basic_data_dict['radius'] = 2* (int(float(data.data)) + 1)

def	PrintRow1(data):
	global yekan
	yekan = int(data.data)
	row_count_dict['yekan'] = yekan

def PrintRow2(data):
	global dahgan
	dahgan = int(data.data)
	row_count_dict['dahgan'] = dahgan
	
def PrintRow3(data):
	global sadgan
	sadgan = int(data.data)
	row_count_dict['sadgan'] = sadgan


def MoveBeadBackPublisher(value, maxRowLength, beadRadius, prevValue=0, rowNum=0):
	nbBead = 10
	maxRowLength = basic_data_dict['length']
	beadRadius = basic_data_dict['radius']
	yekan = row_count_dict['yekan']
	dahgan = row_count_dict['dahgan']
	minRowLength = 0

	for i in range (1, nbBead + 1):
		movingBead = i
		movingLocation = minRowLength + (beadRadius * ( i - 1/2  ))
		position_publish = rospy.Publisher('abacus/row' + str(rowNum) + '/bead'+ str(movingBead) + '/setX', String, queue_size=1)
		position_publish.publish(str(movingLocation))


def MoveBeadPublisherRow2(value, maxRowLength, beadRadius, turn, stay, prevValue=0, rowNum=0):
	"""
	Moves the bead of the second row 
	"""
	nbBead = 10
	maxRowLength = basic_data_dict['length']
	beadRadius = basic_data_dict['radius']
	yekan = row_count_dict['yekan']
	dahgan = row_count_dict['dahgan']
	minRowLength = 0
	bufferLocation = {}

	movingDistance = float(maxRowLength) - float(nbBead) * float(beadRadius)
	movingSlice = float(movingDistance/turn)

	for h in range(1, turn + 2 ):
		PointAtTheTablet("Right", h, turn + 1, 2, stay, 0.2)
		
		for i in range(1, value + 1):
			movingBead = nbBead - i
			if h == 1:
				currentLocation = float(minRowLength) + (float(beadRadius) *  float(movingBead) )
			else:
				currentLocation = bufferLocation[i]

			if h == turn + 1:
				movingLocation = maxRowLength - (beadRadius * (i - 1))
			else:
				movingLocation = float(currentLocation) + float(movingSlice)
			bufferLocation[i] = movingLocation

			# Publishing the positions to the abacus side 
			position_publish = rospy.Publisher('abacus/row' + str(rowNum)+ '/bead'+ str(movingBead) + '/setX', String, queue_size=1)
			position_publish.publish(str(movingLocation))


def MoveBeadPublisher(value, maxRowLength, beadRadius, turn, stay, prevValue=0, rowNum=0):
	"""
	Decides which beads to mve based on the number that is supposed to e shown 
	"""

	# Definition of basic data
	nbBead = 10
	bufferLocation = {}
	
	rospy.Subscriber('/abacus/row' + str(0) + '/count', String, PrintRow1)
	rospy.Subscriber('/abacus/row' + str(1) + '/count', String, PrintRow2)
	#yekan = row_count_dict['yekan']
	#dahgan = row_count_dict['dahgan']
	yekan = prevValue

	minRowLength = 0
	maxRowLength = basic_data_dict['length']
	beadRadius = basic_data_dict['radius']

	# Decides the movement steps based on the turns
	movingDistance = float(maxRowLength) - float(nbBead) * float(beadRadius)
	movingSlice = float(movingDistance/turn)

	if yekan == 0:

		for h in range(1, turn + 2):

			PointAtTheTablet("Right", h, turn + 1, 1, True)

			for i in range(1, value + 1):

				movingBead = nbBead - i

				if h == 1:
					currentLocation = float(minRowLength) + (float(beadRadius) *  float(movingBead) )
				else:
					currentLocation = bufferLocation[i]

				if h == turn + 1:
					movingLocation = maxRowLength - (beadRadius * (i - 1))
				else:
					movingLocation = float(currentLocation) + float(movingSlice)

				bufferLocation[i] = movingLocation

				# Publishing the positions to the abacus side 
				position_publish = rospy.Publisher('abacus/row' + str(rowNum)+ '/bead'+ str(movingBead) + '/setX', String, queue_size=1)
				position_publish.publish(str(movingLocation))

		PointAtTheTablet("Right", h + 1, turn + 2, 1, stay)


	else:
		if value >= 0:
			for h in range(1, turn + 2 ):

				PointAtTheTablet("Right", h, turn + 1, 1, stay)

				for i in range(1, value + 1):

					if yekan + i <= nbBead:

						movingBead = nbBead - (yekan + i)

						if h == 1:
							currentLocation = float(minRowLength) + (float(beadRadius) *  float(movingBead) )
						else:
							currentLocation = bufferLocation[i]

						if h == turn + 1:
							movingLocation = maxRowLength - (beadRadius * (yekan + i - 1))
						else:
							movingLocation = float(currentLocation) + float(movingSlice)

						bufferLocation[i] = movingLocation

						# Publishing the positions to the abacus side 
						position_publish = rospy.Publisher('abacus/row' + str(rowNum) + '/bead'+ str(movingBead) + '/setX', String, queue_size=1)
						position_publish.publish(str(movingLocation))
	
		elif value < 0:
			for h in range(1, turn+1 ):

				PointAtTheTablet("Left", h, turn, 1, stay)

				for i in range(1, (- value) + 1):

					movingBead = nbBead - yekan + i - 1
					
					if h == 1:
						currentLocation = maxRowLength - (beadRadius * (nbBead - movingBead))
					else:
						currentLocation = bufferLocation[i]

					if h == turn :
						movingLocation = minRowLength + (beadRadius * (movingBead + 1)) - round(beadRadius/2)
					else:
						movingLocation = float(currentLocation) - float(movingSlice)

					bufferLocation[i] = movingLocation

					# Publishing the positions to the abacus side 
					position_publish = rospy.Publisher('abacus/row0/bead'+ str(movingBead) + '/setX', String, queue_size=1)
					position_publish.publish(str(movingLocation))



def PointAtTheTablet(direction, step, turn, row, stay, Const = 0):
	"""
	The function that carry out true pointing
	"""
	global tracker
	global frame

	frame = motion.FRAME_ROBOT
	effector = 'RArm'
	maxSpeedHead = 0.2
	useWholeBody = False
	useSensorValues = True
	
	# Initialization of pointing direction
	XA = 0.2

	if Const != 0:
		XB = 0
	else:
		XB = -0.2
	

	Slice = float((XA - XB)/turn)

	if effector == "RArm":
		motionProxy.openHand("RHand")
	else:
		motionProxy.openHand("LHand")

	if direction == "Left":
	 	position1 = [0.3 + row * 0.2, XA - Slice * step, 0.1]
	 	position2 = [0.7, 0, 0.1]
		position3 = [0.7, -0.2, 0.1]
	elif direction == "Right":
		position1 = [0.3 + row * 0.2, XB + Slice * step + Const, 0.1]
		position2 = [0.7, 0, 0.1]
		position3 = [0.7, 0.2, 0.1]

	curPose = motionProxy.getPosition("RHand", frame, useSensorValues)
	#print ("dirrection, step, turn, row, stay", direction, step, turn, row, stay)
	#print "position----------------------------", position1
	# Tracker
	tracker.post.lookAt(position1, frame, maxSpeedHead, useWholeBody)
	tracker.pointAt("RArm", position1, frame, maxSpeedHead)

	# Robot goes back to resting position
	if step == turn and not stay:
		postureProxy.goToPosture("Crouch", 0.3)


def PrintButtonCorrect (data):
	print (data)


def PrintButtonWrong (data):
	print (data)


def RobotSpeaking(instructionText, childName):
	"""
	Adding the speed and the pause to the text recieved from the contol panel
	And then the robot speaking
	"""
	speakingText = "\\rspd=80\\"
	i = 0

	for text in instructionText:
		if text == " Sierra":
			instructionText[i] = childName

		speakingText += instructionText[i] + "\\pau=500\\"
		i += 1

	print " in RobotSpeaking"
	print speakingText
	communicate.say(speakingText)



def AmbigiousBehaviour(currentElements, robotTaskNumber):
	"""
	Ambiguous Behaniour

	"""

	firstValue = currentElements[0]
	secondValue = currentElements[1]
	answerValue = currentElements[2]



	print ("\\rspd=90\\ can you move " + str(secondValue) + " beads to the right?")
	communicate.say (" Can you move " + str(secondValue) + " beads to the right")



def EgocentricBehaviour(currentElements, robotTaskNumber):

	firstValue = currentElements[0]
	secondValue = currentElements[1]
	answerValue = currentElements[2]

	if secondValue < 0:
		correctedSecondValue = secondValue * -1
		direction = "right"

	if secondValue > 0:
		correctedSecondValue = secondValue 
		direction = "left"

	print ("\\rspd=90\\ can you move " + str(correctedSecondValue) + " beads to my " + direction)
	communicate.say (" Can you move " + str(correctedSecondValue) + " beads to my " + direction)


def AddresseeBehaviour(currentElements, robotTaskNumber):

	firstValue = currentElements[0]
	secondValue = currentElements[1]
	answerValue = currentElements[2]

	if secondValue < 0:
		correctedSecondValue = secondValue * -1
		direction = "left"

	if secondValue > 0:
		correctedSecondValue = secondValue 
		direction = "right"

	print ("\\rspd=90\\ can you move " + str(correctedSecondValue) + " beads to your " + direction)
	communicate.say (" Can you move " + str(correctedSecondValue) + " beads to your " + direction)




def faceTrackingStarted(faceSize):
	""" Robot starts to track the users face

	"""

	# First, wake up
	#motionProxy.wakeUp()
	#motionProxy.rest()

	# Add target to track
	targetName = "Face"
	faceWidth = faceSize
	tracker.registerTarget(targetName, faceWidth)

	# Then, start tracker
	tracker.track(targetName)


def faceTrackingEnded():
	""" Robot stops to track the users face and go into resting mode after certain keys are pressed

	"""

	tracker.stopTracker()
	tracker.unregisterAllTargets()
	#motionProxy.rest()


def LookAtTheTablet(pitch_angle=0.3, yaw_angle=0):
	""" Move the robot's head to look at the tablet

	"""
	global fractionMaxSpeed
	motionProxy.setStiffnesses("Head", 1.0)
	#pitch_angle = 0.3
	# Example showing how to set angles, using a fraction of max speed
	isAbsolute = True
	names  = ["HeadYaw", "HeadPitch"]
	angles  = [yaw_angle, pitch_angle]
	#fractionMaxSpeed  = 0.05
	moveTime = 1
	motionProxy.angleInterpolation(names, angles, moveTime, isAbsolute)

	"""names               = "HeadYaw"
	changes             = -0.5
	fractionMaxSpeed    = 0.05
	motionProxy.changeAngles(names, changes, fractionMaxSpeed)"""
	
	global robot_state_pub
	robot_state_pub.publish("Looking at The Book")

	time.sleep(0.5)
	motionProxy.setStiffnesses("Head", 0.0)


def LookAtTheChild(Switch):
	global faceProxy
	# Face tracking activated
	#faceTrackingEnded()
	#faceProxy.enableTracking(tracking_enabled)
	faceSize = 0.1
	if Switch == True:
		facesize = 0.1
		faceTrackingStarted(faceSize)
	elif Switch == False:
		faceTrackingEnded()


def TurnOffCoCounter():

	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', False)
	motionProxy.killAll()



def GoToFinishLine(once, childName):
	print (" in the finish line")

	if once == 1:
		facesize = 0.1
		faceTrackingStarted(facesize)
		global robot_state_pub
		robot_state_pub.publish( taskLevel_dict['CurrLevel'] + " Selected")

		idleMovementModeOFF()
		facesize = 0.1
		global restingEnabled 
		restingEnabled = False
		faceTrackingStarted(facesize)
		
		
		communicate.setLanguage('English')
		communicate.say("\\rspd=80\\ hey " + childName)
		communicate.say("\\rspd=80\\ why don't you reed the rest of the book for me?")
		communicate.say("\\rspd=80\\ I really liked it when you reed it for me last time")
		motionProxy.setBreathEnabled('Arms', True)


def turn_on_eye():
	section1 = ["FaceLeds", "ChestLeds" ]
	proxy.createGroup("turn",section1)
	proxy.fadeRGB("turn", 0x00FFFFFF, 0.3)



def blinkingModeON(mode):

	global blinkThread
	blinkThread = threading.Timer(5, blinkingModeON, [mode])
	emotionReaction = EYEEMOTIONS(proxy)
	animationSelection = MOTION_ANIMATION_SELECTION()

	if mode == "ON":
		blinkThread.start()	
		
		emotionReaction.blink_eyes()
		#animationSelection.reactionIdleMovement()

	if mode == "OFF":
		emotionReaction.turn_off_eye()
		blinkThread.cancel()


def blinkingModeOFF():
	global blinkThread
	#emotionReaction.turn_off_eye()
	blinkThread.cancel()



def IntroduceNao(numOfStart,childName):
	if numOfStart == "1":
		print ("Hello " + childName + "\\pau=700\\ Welcome back \\pau=500\\")
		#communicate.say("\\rspd=90\\ Hello everyone \\pau=700\\ My name is Nao")
		postureProxy.goToPosture("Crouch", 0.2)

current_action_dict = {}



def ChildTurnInteractionScenario(productionBatch, childTaskNumber):
	"""	Robot looks at the child when the child talks
		robot looks at the abacus
		and when the child asks what is the number? 
		robot answers based on the reading from the file
	"""
	
	# Initialization
	nbBead = 10
	turn = 3
	stay = False
	
	
	# CALCULATION
	#######################################################################
	reset_publish = rospy.Publisher('reset/order', String, queue_size=1)
	reset_publish.publish("1")

	# The numbers that the child has in their card
	#-------------------------------------
	currentElements = [int(i) for i in productionBatch[childTaskNumber-1]]
	print "currentElements ......................" , currentElements


	#dict_key = "P " + str(prodSeries) + "&" + str(proOrder) + ": " + str(currentElements)
	#robot_move_dict [dict_key] = "DO NOTHING"

	# Subscribe to the max diatance and the size of the beads recieved from the virtual abacus side
	rospy.Subscriber('/abacus/bead/radius', String, PrintRadius)
	rospy.Subscriber('/abacus/row/length', String, PrintLength)
	
	beadRadius = basic_data_dict['radius']
	maxRowLength = basic_data_dict['length']

	# ACTION (old Scenario)
	#######################################################################
	# The child asks the robot which number the abacus shows?
	# the robot says the right number according to 

	# ACTION 
	#######################################################################
	# Robot looks at the child
	#-------------------------------------
	LookAtTheChild(True)
	print "Look at the Child.......................", "True"
	reset_publish.publish("0")
	#print ("reset")
	#RobotUtterances("First Number")
	

	
	# Child tells the robot to show a number
	# child uses a card to give instruction
	# robot knows that number based on the production_series data set
	# the numbers for each trial is stored in currentElements 
	#------------------------------------
	#time.sleep(5)
	firstValue = currentElements[0]

	# robot acts like it has heard the child, so now it looks at the tablet 
	# and shows the number requested
	# it also moves its hand to point at it 
	LookAtTheChild(False)
	print "Look at the Child.......................", "False"
	time.sleep(1)

	# Wait for control decision message
	# ------------------------------------------
	

	# Moving beads operation
	# ------------------------------------------
	print("robot looking at the tablet")
	LookAtTheTablet()
	MoveBeadPublisher(firstValue, maxRowLength, beadRadius, turn, stay)

	# robot looks at the child again
	#------------------------------------------
	LookAtTheChild(True)
	print "Look at the Child.......................", "True"


def ChildTurnResponseScenario(dict_key, if_wait, childTurnOrder, productionBatch, childTaskNumber):

	currentElements = [int(i) for i in productionBatch[childTaskNumber-1]]
	firstValue = currentElements[0]

	# Initialization
	nbBead = 10
	turn = 3
	stay = False

		# Subscribe to the max diatance and the size of the beads recieved from the virtual abacus side
	rospy.Subscriber('/abacus/bead/radius', String, PrintRadius)
	rospy.Subscriber('/abacus/row/length', String, PrintLength)
	
	beadRadius = basic_data_dict['radius']
	maxRowLength = basic_data_dict['length']

	# CALCULATION
	#######################################################################
	reset_publish = rospy.Publisher('reset/order', String, queue_size=1)

	if current_action_dict [dict_key] == "WAIT":
		RobotUtterances("WAIT")
		#whilecounter = 0


	elif current_action_dict [dict_key] == "ACTION":

		secondValue = currentElements[1]

		# CALCULATION
		##########################################################################
		# Now it is the time to solve the addition or subtraction question
		# in this stage there are 5 trials, 
		# robot answers to the first and fifth ones correctly
		# and it randomly answers to two questions wrongly among the 2nd to 4th ones


		# Making decision on how to make a wrong answer
		# ### If the second value is less than first value, I want the robot to make direction mistake
		# ### If the second value is more than first value, I want the robot to make value mistake, like 
		# moving few more or less beads
		# ---------------------------------------------------- 
		if firstValue > secondValue:
			wrongSecondValue = -1 * secondValue
		else:
			wrongSecondValue = random.choice([x for x in range(secondValue-2, secondValue+3) if x != secondValue])


		print robot_answers_dict

		# robot_answers_dict is a dictionary that provide a random plan of 3 correct and 2 wrong answers for the robot
		# the robot answers based on what is on this dict
		#---------------------------------------------------
		if robot_answers_dict[childTaskNumber]:
			# robot moves correct number of beads
			#print ( secondValue)
			XsecondValue = secondValue
			answerValue = firstValue + secondValue
			print "correct mode"
			print ("Now the answer is " + str(answerValue))
			robot_response = True

		elif not (robot_answers_dict[childTaskNumber]):
			#print (wrongSecondValue)
			XsecondValue = wrongSecondValue
			answerValue = firstValue + wrongSecondValue
			print "wrong mode"
			print ("Now the answer is " + str(answerValue))
			robot_response = False
		
		
		if if_wait == True:
			RobotUtterances("IF WAIT")

		# ACTION
		###########################################################################
		# robot moves the second value based on the "order of trial" which means right or worng answers
		LookAtTheChild(False)
		print "Look at the Child.......................", "False"
		time.sleep(2)

		LookAtTheTablet()
		# When the answer is less than 10
		#--------------------------------------------
		if XsecondValue + firstValue < nbBead:
			stay = False
			MoveBeadPublisher(XsecondValue, maxRowLength, beadRadius, turn, stay, firstValue)

		# When the answer is more than 10
		#--------------------------------------------
		if XsecondValue + firstValue >= nbBead:
			belowTen = nbBead - firstValue
			stay = True
			# Move The beads in the first row to get 10
			MoveBeadPublisher(belowTen, maxRowLength, beadRadius, turn, stay, firstValue )
			time.sleep(0.5)
			
			# Move the beads back to initial position in the first row
			PointAtTheTablet("Left", 1, 2, 1, stay, -0.01)
			reset_publish.publish("1")
			PointAtTheTablet("Left", 2, 2, 1, stay, -0.01)
			time.sleep(0.5)
			
			# Move one bead from the second row to count the 10
			MoveBeadPublisherRow2(1, maxRowLength, beadRadius, turn, stay, 0, 1)
			AboveTen = XsecondValue - belowTen
			time.sleep(1.5)

			# Move the rest of the beads in the first row to find the final answer
			stay = False
			MoveBeadPublisher(AboveTen, maxRowLength, beadRadius, turn, stay)

		time.sleep(1)

		# At this point the robot tells the final answer to the child
		LookAtTheChild(True)
		print "Look at the Child.......................", "True"
		communicate.say ("The answer is " + str(answerValue))
		RobotUtterances("After Finishing")
		time.sleep(2)

		print "firstValue -------------- " + str(firstValue)
		print "secondValue ------------- " + str(secondValue)
		print "wrongSecondValue -------- " + str(wrongSecondValue)
		print "answerValue ------------- " + str(answerValue)


		# Subscribe to the event of child's evaluation of the robot's answer
		# Child's response will come from the abacus side
		#----------------------------------------------------------------
		rospy.Subscriber('/abacus/button/correct', String, PrintButtonCorrect)
		rospy.Subscriber('/abacus/button/wrong', String, PrintButtonWrong)
		child_response = True

		if child_response and robot_response:
			answerCondition = "TruePositive"
		elif not child_response and robot_response:
			answerCondition = "FalsePositive"
		elif child_response and not robot_response:
			answerCondition = "FalseNegative"
		elif not child_response and not robot_response:
			answerCondition = "TrueNegative"


		confusion_matrix_dict ["P " + str(childTurnOrder) + "&" + str(childTaskNumber) + ": " + str(currentElements)] = answerCondition

		# CONDITIONS
		#######################################################################################
		# Child recognizes the robot's mistake or not
		# There are four conditions
		# 1. Robot makes no mistake, child acknowledges that Condition = True Positive condition=TruePositive
		# 2. Robot makes no mistake, but the child thinks the robot did = False Positive condition=FalsePositive
		# 3. Robot makes mistake, child doesn't recognize it = False Negative condition=FalseNegative
		# 4. Robot makes mistake, child recognizes it = True Negative condition=TrueNegative


		# Recieves Finish Task Signal

		# Robot reacts based on the child's evaluation
		#---------------------------------------------------
		#if child_response:
		#	communicate.say ("I am so happy?")
		#elif not child_response:
		#	communicate.say ("I am sorry, I will try to be more careful")


		if childTaskNumber == 4:
			communicate.say ("I sloved a lot of questions, now it is your turn to solve my questions")

			if childTurnOrder == 1:
				initialization_dict["robotTurnCondition"] = 1
				initialization_dict["robotTaskNumber"] = 1

			if childTurnOrder == 2:
				initialization_dict["robotTurnCondition"] = 2
				initialization_dict["robotTaskNumber"] = 1

			if childTurnOrder == 3:
				initialization_dict["endOfInteraction"] = 1

		else:
			RobotUtterances("Move Next")



def ChildTurnPreparation(childTurnOrder):
	
	productionBatch = []
	
	production_numbers = rospy.get_param('~production_numbers_series1')
	numbersArray = production_numbers.splitlines()
	#print numbersArray

	# Seperate the numbers in each line
	elements = []
	for line in numbersArray:
		elements.append(re.split('\W+,', line))
	elements = elements[2:]

	if childTurnOrder == 1:
		productionBatch = elements[(childTurnOrder - 1)*5: (childTurnOrder)*5]
		robot_answers_dict[1] = True
		robot_answers_dict[2] = False
		robot_answers_dict[3] = True
		robot_answers_dict[4] = False

	if childTurnOrder == 2:
		productionBatch = elements[(childTurnOrder - 1)*5 + 1 : (childTurnOrder)*5 + 1]
		robot_answers_dict[1] = True
		robot_answers_dict[2] = False
		robot_answers_dict[3] = False
		robot_answers_dict[4] = True

	if childTurnOrder == 3:
		productionBatch = elements[(childTurnOrder - 1)*5 + 2: (childTurnOrder)*5 + 2]
		robot_answers_dict[1] = True
		robot_answers_dict[2] = False
		robot_answers_dict[3] = True
		robot_answers_dict[4] = True

	if childTurnOrder == 4:
		productionBatch = elements[(childTurnOrder - 1)*5 + 3: (childTurnOrder)*5 + 3]
		robot_answers_dict[1] = True
		robot_answers_dict[2] = True
		robot_answers_dict[3] = True
		robot_answers_dict[4] = True

	#TrialBehaviour()
	#robot_answers_dict[1] = True
	#robot_answers_dict[2] = True

	for key in sorted(confusion_matrix_dict.iterkeys()):
	    #print "%s: %s" % (key, confusion_matrix_dict[key])
	    confusin_mat_prod_pub.publish( key + "  :"+ confusion_matrix_dict[key])

	#finishProTask = True
	#print productionBatch
	return productionBatch


def RobotUtterances(interactionPoint):
	"""

	"""
	randomNum = random.randint(1,3)

	if randomNum == 1:
		firstWords = "\\rspd=80\\ I'm ready"
		waitWords = "\\rspd=80\\ Sorry, \\pau=50\\ but I don't know what to do"
		ifwaitWords = "\\rspd=80\\ Aha , \\pau=50\\ you mean like this?"
		secondWords = ""
		lastwords = "\\rspd=80\\ I hope that \\pau=50\\ I did a good job"
		nextwords = "\\rspd=80\\ Let's do the next question"


	if randomNum == 2:
		firstWords = "\\rspd=80\\ Let's go"
		waitWords = "\\rspd=80\\ Sorry, \\pau=50\\ I don't understand"
		ifwaitWords = "\\rspd=80\\ Now I understand"
		secondWords = ""
		lastwords = "\\rspd=80\\ I feel good about this"
		nextwords = "\\rspd=80\\ moving to the next question"



	if randomNum == 3:
		firstWords = "\\rspd=80\\ I am ready"
		waitWords = "\\rspd=80\\ wait, \\pau=50\\ you mean I should move the beads?"
		ifwaitWords = "\\rspd=80\\ you mean like this?"
		secondWords = ""
		lastwords = "\\rspd=70\\ I am ready \\pau=50\\ for the next one"
		nextwords = "\\rspd=80\\ "



	if interactionPoint == "First Number":
		communicate.say(firstWords)

	elif interactionPoint == "WAIT":
		communicate.say(waitWords)

	elif interactionPoint == "IF WAIT":
		communicate.say(ifwaitWords)

	elif interactionPoint == "Second Number":
		communicate.say(secondWords)


	elif interactionPoint == "After Finishing":
		communicate.say(lastwords)

	elif interactionPoint == "Move Next":
		communicate.say(nextwords)

def RobotTurnPreparation(robotTurnOrder):

	productionBatch = []
	
	# CALCULATION
	#######################################################################################
	comprehension_numbers = rospy.get_param('~comprehension_numbers_series1')
	numbersArray = comprehension_numbers.splitlines()
	#print numbersArray
	comprehensionBatch =[]

	# Seperate the numbers in each line
	elements = []
	for line in numbersArray:
		elements.append(re.split('\W+,', line))
	elements = elements[2:]


	# CONDITIONS
	#####################################################################################
	if robotTurnOrder == 1:
		comprehensionBatch = elements[(robotTurnOrder - 1)*5: (robotTurnOrder)*5]

	elif robotTurnOrder == 2:
		comprehensionBatch = elements[(robotTurnOrder - 1)*5 + 1: (robotTurnOrder)*5 + 1]

	elif robotTurnOrder == 3:
		comprehensionBatch = elements[(robotTurnOrder - 1)*5 + 2: (robotTurnOrder)*5 + 2]

	for key in sorted(confusion_matrix_dict.iterkeys()):
	    #print "%s: %s" % (key, confusion_matrix_dict[key])
	    confusin_mat_prod_pub.publish( key + "  :"+ confusion_matrix_dict[key])

	return comprehensionBatch

def robotTurnConditionSelector():

	groupCode = initialization_dict["groupCode"]
	robotTurnOrder= initialization_dict["robotTurnCondition"]
	ConditionType = "ambiguous"

	if groupCode == "AE":

		if robotTurnOrder == 1:
			ConditionType = "egocentric"

		elif robotTurnOrder == 2:
			ConditionType == "addressee" 

	if groupCode == "AD":
		
		if robotTurnOrder == 1:
			ConditionType = "ambiguous"

		elif robotTurnOrder == 2:
			ConditionType == "addressee" 

	if groupCode == "EA":
		
		if robotTurnOrder == 1:
			ConditionType = "egocentric"

		elif robotTurnOrder == 2:
			ConditionType == "ambiguous" 

	if groupCode == "DA":
		
		if robotTurnOrder == 1:
			ConditionType = "addressee"

		elif robotTurnOrder == 2:
			ConditionType == "ambiguous" 

	return ConditionType

def RobotTurnInteractionScenario(comprehensionBatch, ConditionType, robotTaskNumber):

	# Initializations
	global confusion_mat_comp_pub	
	global yekan
	global dahgan
	global sadgan

	# The numbers that the robot asks to the child
	#--------------------------------------------------- 
	currentElements = [int(i) for i in comprehensionBatch[robotTaskNumber]]
	print "currentElements ......................" , currentElements
	firstValue = currentElements[0]
	secondValue = currentElements[1]
	answerValue = currentElements[2]

	# Interaction
	#-----------------------------------------------------
	print (" First, show me " + str(firstValue) + " on the abacus")
	communicate.say ("\\rspd=80\\ can you show me \\pau=50\\" + str(firstValue) + " on the abacus?")
	communicate.say ("\\rspd=80\\ can you show me \\pau=50\\" + str(firstValue) + " on the abacus?")
	

	# Check on the child answer that comes from the abacus
	# Recieves the current count from the abacus side of the code 
	#------------------------------------------------------
	rospy.Subscriber('/abacus/row' + str(0) + '/count', String, PrintRow1)
	rospy.Subscriber('/abacus/row' + str(1) + '/count', String, PrintRow2)
	rospy.Subscriber('/abacus/row' + str(2) + '/count', String, PrintRow3)

	yekan = row_count_dict['yekan']
	dahgan = row_count_dict['dahgan']
	sadgan = row_count_dict['sadgan']

	#LookAtTheChild(False)
	#print "Look at the Child.......................", "False"
	#time.sleep(1)
	
	LookAtTheTablet()
	time.sleep(3)
	LookAtTheTablet(-0.1)

	if dahgan == 0:
		child_first_value = yekan
	elif dahgan > 0:
		child_first_value = dahgan * 10 + yekan


	#child_first_value = firstValue
	if child_first_value == firstValue:
		levelOneCorrection = True
	else:
		levelOneCorrection = False

	time.sleep(2)
	#LookAtTheChild(True)
	#print "Look at the Child.......................", "True"
	

def RobotTurnResponseScenario(dict_key, robotTurnOrder, comprehensionBatch, ConditionType, robotTaskNumber):


	# Initializations
	global confusion_mat_comp_pub	
	global yekan
	global dahgan
	global sadgan

	# The numbers that the robot asks to the child
	#--------------------------------------------------- 
	currentElements = [int(i) for i in comprehensionBatch[robotTaskNumber]]
	print "currentElements ......................" , currentElements
	firstValue = currentElements[0]
	secondValue = currentElements[1]
	answerValue = currentElements[2]
	

	# Check on the child answer that comes from the abacus
	# Recieves the current count from the abacus side of the code 
	#------------------------------------------------------
	rospy.Subscriber('/abacus/row' + str(0) + '/count', String, PrintRow1)
	rospy.Subscriber('/abacus/row' + str(1) + '/count', String, PrintRow2)
	rospy.Subscriber('/abacus/row' + str(2) + '/count', String, PrintRow3)

	yekan = row_count_dict['yekan']
	dahgan = row_count_dict['dahgan']
	sadgan = row_count_dict['sadgan']

	if dahgan == 0:
		child_first_value = yekan
	elif dahgan > 0:
		child_first_value = dahgan * 10 + yekan


	#child_first_value = firstValue
	if child_first_value == firstValue:
		levelOneCorrection = True
	else:
		levelOneCorrection = False


	if ConditionType == "ambiguous":
		AmbigiousBehaviour(currentElements, robotTaskNumber)


	if ConditionType == "egocentric":
		EgocentricBehaviour(currentElements, robotTaskNumber)


	if ConditionType == "addressee":
		AddresseeBehaviour(currentElements, robotTaskNumber)

	#print ("\\rspd=90\\ can you move " + str(secondValue) + " beads to the right?")
	#communicate.say (" Can you move " + str(secondValue) + " to the right")

	child_second_value = secondValue
	
	if child_second_value == secondValue:
		levelTwoCorrection = True
	else:
		levelTwoCorrection = False


	print (" Good Job")
	print ("what is the answer?")

	child_answer_value = answerValue
	if child_answer_value == answerValue:
		levelThreeCorrection = True
	else:
		levelThreeCorrection = False


	confusion_mat_comp_pub.publish("levelOneCorrection :"+ str(levelOneCorrection))
	confusion_mat_comp_pub.publish("levelTwoCorrection :"+ str(levelTwoCorrection))
	confusion_mat_comp_pub.publish("levelThreeCorrection :"+ str(levelThreeCorrection))




	# Subscribe to an event
	# Child's response will come from the abacus
	# rospy.Subscriber('child_response', Bool)
	child_response = True
	robot_response = True
	if child_response and robot_response:
		answerCondition = "TruePositive"
	elif not child_response and robot_response:
		answerCondition = "FalsePositive"
	elif child_response and not robot_response:
		answerCondition = "FalseNegative"
	elif not child_response and not robot_response:
		answerCondition = "TrueNegative"



	if robotTaskNumber == 4:
		communicate.say ("How about we switch again?")
		communicate.say ("I want to solve some questions?")

		if robotTurnOrder == 1:
			initialization_dict["childTurnCondition"] = 2
			initialization_dict["childTaskNumber"] = 1

		if robotTurnOrder == 2:
			initialization_dict["childTurnCondition"] = 3
			initialization_dict["childTaskNumber"] = 1


	#confusion_matrix_dict ["C " + "ambiguous" + "&" + str(robotTaskNumber) + ": " + str(currentElements)] = answerCondition

def robotLooking(side):
	frame = motion.FRAME_ROBOT
	maxSpeedHead = 0.2
	useWholeBody = False
	useSensorValues = True
	LookAtTheChild(False)

	if side == " right":

		position2 = [0.7, 0, 0.1]
		effector = 'RArm'
		#motionProxy.openHand("RHand")
		#tracker.post.lookAt(position2, frame, maxSpeedHead, useWholeBody)
		LookAtTheTablet(0.2, -0.25)
		tracker.pointAt("RArm", position2, frame, maxSpeedHead)

		#
		time.sleep(2)
		#LookAtTheTablet(-0.1, 0)

		#motionProxy.closeHand("RHand")
		postureProxy.goToPosture("Crouch", 0.3)

	if side == " left":
		position2 = [0.7, 0, 0.1]

		effector = 'LArm'
		#motionProxy.openHand("LHand")

		LookAtTheTablet(0.2, 0.25)
		tracker.pointAt("LArm", position2, frame, maxSpeedHead)

		time.sleep(2)
		

		#motionProxy.closeHand("LHand")
		postureProxy.goToPosture("Crouch", 0.3)

#def DefineObjectZone():



	
def MakeInstruction():
	if color == 'yellow':
		0,1,2

	if color == 'red':
		3,4,5

	if color == 'blue':
		6,7,8

global completeInstruction
completeInstruction = []
global instructionOrder
instructionOrder = []

obejcts_positions_dict={
}

global num 
num = 0
def SaveObjectsPositions():
	global num
	collect_publish = rospy.Publisher('collect/positions', String, queue_size=1)
	
	num += 1 
	collect_publish.publish(str(num))
	print "num", num
	# for box = 0
	box_number = "0"
	object_type = "ball"
	for i in range(9):
		for  object_type in ["ball", "square"]:
			key0 = "/box" + box_number + "/" + object_type + str(i) + "/xchange"
			#print key0
			rospy.Subscriber(key0, String, PrintLocationToDic, key0)

	key2 = "/box" + box_number + "/width"
	rospy.Subscriber(key2, String, PrintLocationToDic, key2)
	
	box_number = "1"
	for i in range(9):
		for  object_type in ["ball", "square"]:
			key1 = "/box" + box_number + "/" + object_type + str(i) + "/xchange"
			#print key1
			rospy.Subscriber(key1, String, PrintLocationToDic, key1)

	key2 = "/box" + box_number + "/width"
	rospy.Subscriber(key2, String, PrintLocationToDic, key2)

	#print obejcts_positions_dict



def PrintLocationToDic(data, key):
	obejcts_positions_dict[key] = data.data
	



def ClassifyTheInstructions(instructionOrder, completeInstruction, activeColumn):

	for i in range(3):
		if instructionOrder[i] == "COLOR":
			moveColor = completeInstruction[i]
			#DecideColor(moveColor)
		if instructionOrder[i] == "OBJECT":
			moveObject = completeInstruction[i]

		if instructionOrder[i] == "DIREC":
			moveSide = completeInstruction[i]

	# For when the instructions come from the child or are for the child
	if activeColumn == "Child_Turn":
		row_number = "0"
	elif activeColumn == "Robot_Turn":
		row_number = "1"


	# For when the object type id defined
	if moveObject[0] == " ball":
		object_type = "ball"
	elif moveObject[0] == " square":
		object_type = "square"
	#elif moveObject[0] == " object":
	#	object_type = "both" 

	# For color selection
	if moveColor[0] == " yellow":
		object_number = [0, 1, 2]
	elif moveColor[0] == " red":
		object_number = [3, 4, 5]
	elif moveColor[0] == " blue":
		object_number = [6, 7, 8]


	# For direction selection
	if moveSide[0] == " robot LEFT" or moveSide[0] == " child RIGHT":
		object_side = "firsthalf"
	elif moveSide[0] == " robot RIGHT" or moveSide[0] == " child LEFT":
		object_side = "secondhalf"
	elif moveSide[0] == " the LEFT" or moveSide[0] == " the RIGHT":
		object_side = "clarification"


	print "moveColor, moveObject, moveSide"
	print moveColor, moveObject, moveSide

	print "row_number, object_type, object_number, object_side"
	print row_number, object_type, object_number, object_side
	#SaveObjectsPositions(row_number, object_number, object_type)

	#print obejcts_positions_dict
	key2 = "/box0/width"
	halfLine = obejcts_positions_dict ["/box0/width"]

	PerformTheInstruction(row_number, object_type, object_number, object_side)

def PerformTheInstruction(box_number, object_type, object_number, object_side):

	moving_objects_x = []
	child_right_box = []
	child_left_box = []
	robot_right_box = []
	robot_left_box = []

	if box_number == "0":	
		midPoint = float(obejcts_positions_dict ["/box" + box_number + "/width"])/2
		for i in object_number:
			key = "/box" + box_number + "/" + object_type + str(i) + "/xchange"
			key_pub = "game/box" + box_number + "/" + object_type + str(i)  + "/setX"
			moving_objects_x.append(obejcts_positions_dict [key])
			if float(obejcts_positions_dict [key]) <= midPoint:
				child_right_box.append(key)
			else:
				child_left_box.append(key)

	if box_number == "1":	
		midPoint = float(obejcts_positions_dict ["/box" + box_number + "/width"])/2
		for i in object_number:
			key = "/box" + box_number + "/" + object_type + str(i) + "/xchange"
			key_pub = "game/box" + box_number + "/" + object_type + str(i)  + "/setX"
			moving_objects_x.append(obejcts_positions_dict [key])
			if float(obejcts_positions_dict [key]) <= midPoint:
				robot_left_box.append(key)
			else:
				robot_right_box.append(key)


	print midPoint
	print moving_objects_x
	print child_right_box
	print child_left_box 
	print robot_right_box
	print robot_left_box
	position_publish = rospy.Publisher('abacus/row' + str(rowNum)+ '/bead'+ str(movingBead) + '/setX', String, queue_size=1)

	if object_side == "firsthalf":
		for key in robot_right_box:
			position_publish = rospy.Publisher('abacus/row' + str(rowNum)+ '/bead'+ str(movingBead) + '/setX', String, queue_size=1)
			position_publish.publish( float(obejcts_positions_dict [key]) - midPoint )
			print "firsthalf"

		for key in child_left_box:
			position_publish.publish( float(obejcts_positions_dict [key]) - midPoint )
			print "firsthalf"

	if object_side == "secondhalf":
		for key in robot_left_box:
			position_publish.publish( midPoint + float(obejcts_positions_dict [key]) )
			print "secondhalf"

		for key in child_right_box:
			position_publish.publish( midPoint + float(obejcts_positions_dict [key]) )
			print "secondhalf"





def DecisionMakingThread(data):
	"""
	This thread receives the instructions from the other side
	"""

	# GLOBALS
	global instructionText
	global reset_publish
	global turn_publish
	global if_wait
	global completeInstruction
	global instructionOrder
	reset_publish = rospy.Publisher('reset/order', String, queue_size=1)
	turn_publish = rospy.Publisher('game/turn', String, queue_size=1)

	# Initialization
	#------------------------------------------------
	numOfStart = 1
	elements = []
	instructionText = []

	# Processing
	#-------------------------------------------------
	content = data.data
	elements.append( re.split('\W+,', content) )

	# Seperating the contents
	activeColumn = elements[0][0]
	instructionPart = elements [0][1:]
	status = instructionPart[0]
	instructionText = instructionPart[1:]
	print "activeColumn, status, In ButtonPressed"
	print activeColumn, status, instructionText
	print "Group Code:", initialization_dict["groupCode"]

	dict_key = 	"G" + str(initialization_dict["groupCode"]) + \
				"C" + str(initialization_dict["childTurnCondition"]) + \
				"T" + str(initialization_dict["childTaskNumber"]) + \
				"R" + str(initialization_dict["robotTurnCondition"]) + \
				"T" + str(initialization_dict["robotTaskNumber"]) 

	

	childTurnOrder = initialization_dict["childTurnCondition"]
	childTaskNumber = initialization_dict["childTaskNumber"]

	groupCode = initialization_dict["groupCode"]

	productionBatch = ChildTurnPreparation(childTurnOrder)
	print "productionBatch", productionBatch

	robotTurnOrder = initialization_dict["robotTurnCondition"]
	robotTaskNumber = initialization_dict["robotTaskNumber"]
	print "WTF"
	comprehensionBatch = RobotTurnPreparation(robotTurnOrder)
	
	ConditionType = robotTurnConditionSelector()
	print "Dict Key.........................", dict_key
	print "current_action_dict..............", current_action_dict
	print "status", status

	if activeColumn == "Child_Turn":

		if status == " SETUP":
			current_action_dict [dict_key] = "SETUP"
			print "Current Action..................................", current_action_dict [dict_key]
			turn_publish.publish("child")

		if status == " NUM":
			current_action_dict [dict_key] = "NUM"
			current_action_dict ["if_wait"] = "WAIT"
			print "Current Action..................................", current_action_dict [dict_key]
			#ChildTurnResponseScenario(dict_key, if_wait, childTurnOrder, productionBatch, initialization_dict["childTaskNumber"])
		
		if status == " NEXT":
			current_action_dict [dict_key] = "NEXT"
			print "Current Action..................................", current_action_dict [dict_key]

		


		if status == " DIREC":
			current_action_dict [dict_key] = "DIREC"
			print "Current Action..................................", current_action_dict [dict_key]
			if not current_action_dict [dict_key] in instructionOrder:
				completeInstruction.append(instructionText)
				instructionOrder.append(current_action_dict [dict_key])
			#ChildTurnInteractionScenario(productionBatch, initialization_dict["childTaskNumber"])

		if status == " OBJECT":
			current_action_dict [dict_key] = "OBJECT"
			print "Current Action..................................", current_action_dict [dict_key]
			if not current_action_dict [dict_key] in instructionOrder:
				completeInstruction.append(instructionText)
				instructionOrder.append(current_action_dict [dict_key])

		if status == " COLOR":
			current_action_dict [dict_key] = "COLOR"
			print "Current Action..................................", current_action_dict [dict_key]
			if not current_action_dict [dict_key] in instructionOrder:
				completeInstruction.append(instructionText)
				instructionOrder.append(current_action_dict [dict_key])

		



		if status == " ACTION":
			if current_action_dict["if_wait"] == "WAIT":
				if_wait = True
				current_action_dict["if_wait"] = "NULL"
			current_action_dict [dict_key] = "ACTION"
			print "Current Action..................................", current_action_dict [dict_key]
			ChildTurnResponseScenario(dict_key, if_wait, childTurnOrder, productionBatch, initialization_dict["childTaskNumber"])
			if_wait = False
			initialization_dict["childTaskNumber"] += 1

		if status == " RESET":
			#PointAtTheTablet("Left", 1, 2, 1, True, -0.01)
			reset_publish.publish("1")
			#PointAtTheTablet("Left", 2, 2, 1, False, -0.01)
			#reset_publish.publish("0")
			SaveObjectsPositions()
			print obejcts_positions_dict["/box0/ball0/xchange"]

		if status == " STAT":
			current_action_dict [dict_key] = "STATEMENT"
			print "Current Action..................................", current_action_dict [dict_key]
			RobotSpeaking(instructionText, initialization_dict["ChildName"])

		if status == " LOOK":
			current_action_dict [dict_key] = "LOOK"
			print "Current Action..................................", current_action_dict [dict_key]
			robotLooking(instructionText[0])
			print instructionText

		if status == " FINISH":
			current_action_dict [dict_key] = "FINISH"
			print "Current Action..................................", current_action_dict [dict_key]
			initialization_dict["robotTurnCondition"] = 1
			initialization_dict["robotTaskNumber"] = 1

		print "childTaskNumber...............................", initialization_dict["childTaskNumber"]



	elif activeColumn == "Robot_Turn":

		if status == " SETUP":
			current_action_dict [dict_key] = "SETUP"
			print "Current Action..................................", current_action_dict [dict_key]
			turn_publish.publish("robot")

		if status == " START":
			current_action_dict [dict_key] = "START"
			print "Current Action..................................", current_action_dict [dict_key]
			print "Condition Type----------------------------------", ConditionType
			RobotTurnInteractionScenario(comprehensionBatch, ConditionType, initialization_dict["robotTaskNumber"])

		if status == " WAIT":
			current_action_dict [dict_key] = "WAIT"
			print "Current Action..................................", current_action_dict [dict_key]
			RobotSpeaking(instructionText, initialization_dict["ChildName"])

		if status == " ACTION":
			current_action_dict [dict_key] = "ACTION"
			print "Current Action..................................", current_action_dict [dict_key]
			print "Condition Type----------------------------------", ConditionType
			RobotTurnResponseScenario(dict_key, robotTurnOrder, comprehensionBatch, ConditionType, initialization_dict["robotTaskNumber"])
			initialization_dict["robotTaskNumber"] += 1


		if status == " CHECK":
			current_action_dict [dict_key] = "CHECK"
			print "Current Action..................................", current_action_dict [dict_key]
			RobotSpeaking(instructionText, initialization_dict["ChildName"])

		if status == " ANSWER":
			current_action_dict [dict_key] = "CHECK"
			print "Current Action..................................", current_action_dict [dict_key]
			RobotSpeaking(instructionText, initialization_dict["ChildName"])

		if status == " NEXT":
			current_action_dict [dict_key] = "NEXT"
			print "Current Action..................................", current_action_dict [dict_key]


		if status == " RESET":
			PointAtTheTablet("Left", 1, 2, 1, True, -0.01)
			reset_publish.publish("1")
			PointAtTheTablet("Left", 2, 2, 1, False, -0.01)

		if status == " STATEMENT":
			current_action_dict [dict_key] = "STATEMENT"
			print "Current Action..................................", current_action_dict [dict_key]
			RobotSpeaking(instructionText, initialization_dict["ChildName"])

		if status == " FINISH":
			current_action_dict [dict_key] = "FINISH"
			print "Current Action..................................", current_action_dict [dict_key]
			initialization_dict["robotTurnCondition"] = robotTurnOrder + 1
			initialization_dict["robotTaskNumber"] = 1

		print "robotTaskNumber...............................", initialization_dict["robotTaskNumber"]

	elif activeColumn == "Extra_Behavior":
		if status == " LOOK AT CHILD":
			print "child"

		if status == " LOOK AT ABACUS":
			print "abacus"

	if len(completeInstruction) == 3:
		ClassifyTheInstructions(instructionOrder, completeInstruction, activeColumn)
		completeInstruction = []
		instructionOrder = []

	SaveObjectsPositions()
	#print obejcts_positions_dict["/box0/ball0/xchange"]
	LookAtTheChild(True)
	print "Look at the Child.......................", "False"





def main():

	# Initiate the node that does the main computing
	rospy.init_node('main_scenario')

	
	# Inititate the publishers


	global confusin_mat_prod_pub
	confusin_mat_prod_pub = rospy.Publisher("confusion_matrix_production", String, queue_size=5)

	global confusion_mat_comp_pub
	confusion_mat_comp_pub = rospy.Publisher("confusion_matrix_comprehension", String, queue_size=5)


	global robot_state_pub
	robot_state_pub = rospy.Publisher("robot_state", String, queue_size=5)



	global child_name_pub
	child_name_pub = rospy.Publisher("child_name", String, queue_size=1)

	global enabled_state_pub
	enabled_state_pub = rospy.Publisher("enabled_state", String, queue_size=5)

	global robot_instruction_pub
	robot_instruction_pub = rospy.Publisher("instruction_state", String, queue_size=100)

	global robot_gesture_pub
	robot_gesture_pub = rospy.Publisher("gesture_state", String, queue_size=5)

	global page_state_pub
	page_state_pub = rospy.Publisher("page_state", String, queue_size=5)

	global correction_state_pub
	correction_state_pub = rospy.Publisher("correction_state", String, queue_size=1)

	global robot_state_time_pub
	robot_state_time_pub = rospy.Publisher("robot_state_time", Pose, queue_size=5)

	global current_word_pub
	current_word_pub = rospy.Publisher("current_word", String, queue_size=5)

	current_word_time_pub = rospy.Publisher("current_word_time", Pose, queue_size=5)

	global detected_buttons_pub
	detected_buttons_pub = rospy.Publisher("detected_buttons", String, queue_size=5)

	global robot_hand_pose_pub
	robot_hand_pose_pub = rospy.Publisher("robot_hand_pose", Pose, queue_size=10)
	
	global robot_hand_pose_time_pub
	robot_hand_pose_time_pub = rospy.Publisher("robot_hand_pose_time", Pose, queue_size=5)

	global robot_speed_pub
	robot_speed_pub = rospy.Publisher("robot_read_speed", String, queue_size=5)

	global detected_ARtags_pub
	detected_ARtags_pub = rospy.Publisher("detected_ARtags", String, queue_size=5)



	global position_publish
	for i in range(0, 10):
		position_publish = rospy.Publisher('abacus/row0/bead'+ str(i) + '/setX', String, queue_size=1)
		position_publish = rospy.Publisher('abacus/row1/bead'+ str(i) + '/setX', String, queue_size=1)
		position_publish = rospy.Publisher('abacus/row2/bead'+ str(i) + '/setX', String, queue_size=1)


	rospy.Subscriber('/abacus/row/length', String, PrintLength)
	rospy.Subscriber('/abacus/bead/radius', String, PrintRadius)

	# Define Nao IP and inititate all yhe Nao related API
	#nao_IP = rospy.get_param('~nao_ip')
	#nao_IP = 'nao.local'
	nao_IP = '192.168.0.100'
	nao_port = 9559
	
	
	myBroker = ALBroker("myBroker",        
		"0.0.0.0",   # listen to anyone
    	0,           # find a free port and use it
    	nao_IP,         # parent broker IP
    	9559)       # parent broker port)

	#myBroker.shutdown()	

	
	global communicate
	communicate = ALProxy("ALTextToSpeech", nao_IP, nao_port)
	communicate.setLanguage('English')
	communicate.setVolume(1)


	global faceProxy
	faceProxy = ALProxy("ALFaceDetection", nao_IP, nao_port)

	global tracker
	tracker = ALProxy("ALTracker", nao_IP, nao_port)

	global motionProxy
	motionProxy = ALProxy("ALMotion", nao_IP, nao_port)


	global postureProxy
	postureProxy = ALProxy("ALRobotPosture", nao_IP, 9559)


	#global conversration
	#conversation = ALProxy("ALTextToSpeech", nao_IP, 9559)




	
	global audiomotionProxy
	audiomotionProxy = ALProxy("ALAudioPlayer", nao_IP, 9559)
	


	global proxy
	proxy = ALProxy("ALLeds", nao_IP, 9559)

	global memory
	memory = ALProxy("ALMemory", nao_IP, 9559)

	global audioReactionProxy
	audioReactionProxy = ALProxy("ALAudioPlayer", nao_IP, 9559)


	#global SpeechCondition 
	#SpeechCondition = SpeechModule("SpeechCondition")

	subscriber = memory.subscribeToEvent("ALTextToSpeech/TextDone", "SpeechCondition", "onTextDone")


	global maxSpeedHead

	initialiazation_file = rospy.get_param('~initialization_file')
	#with open('./TextFiles/customized_instruction.txt') as f:
	instructionText = initialiazation_file

	#with open('./TextFiles/customized_instruction.txt') as f:
		#instructionText = f.read()
	instructionArray = initialiazation_file.splitlines()

	numOfStart = instructionArray[1]
	initialization_dict["ChildName"] = instructionArray[3]
	initialization_dict["groupCode"] = instructionArray[5]
	initialization_dict["childTurnCondition"] = int(instructionArray[7])
	initialization_dict["robotTurnCondition"] = int(instructionArray[9])
	initialization_dict["childTaskNumber"] = int(instructionArray[11])
	initialization_dict["robotTaskNumber"] = int(instructionArray[13])
	initialization_dict["WhichPage"] = instructionArray[15]
	initialization_dict["CorrectionState"] = instructionArray[17]
	initialization_dict["BookSelection"] = instructionArray[19]

	working_dict = dict(initialization_dict)
	#print working_dict["groupCode"]
	# Initialize the log file
	logFile = logger()
	#time.sleep(5)
	robot_instruction_pub.publish(instructionText)
	time.sleep(1)

	childName = initialization_dict["ChildName"]
	child_name_pub.publish(childName)
	correction_state_pub.publish("correct")

	facesize = 0.1
	global restingEnabled 
	restingEnabled = False
	
	global space
	space      = motion.FRAME_ROBOT

	global RobotConnected
	RobotConnected = True

	# The function to introduce Nao and stop the face tracking 
	IntroduceNao(numOfStart, childName)

	# Subscribe to the tags
	#rospy.Subscriber('tag_id_state', String, tagDetection)

	# Something that can be stated in the initializationFile
	groupCode = "AE"
	orderCounter = 0
	productionSeries = 0
	##### Start the assignment of tasks
	#TaskAssignment(groupCode, orderCounter, productionSeries)

	current_action_dict ["if_wait"] = "NULL"

	rospy.Subscriber('/control/button/content', String, DecisionMakingThread, queue_size=1)

if __name__ == "__main__":

	global communicate
	
	try:
		main()
		rospy.spin()

    	except rospy.ROSInterruptException:
        	print
        	print "Interrupted by user, shutting down"
        	#communicate.stopAll()
        	rospy.loginfo("AR Tag Tracker node terminated.")
        	myBroker.shutdown()
        	sys.exit(0)

