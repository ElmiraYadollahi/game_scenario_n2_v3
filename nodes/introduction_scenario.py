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
import animations.winner_seated_pose_left_arm
import animations.winner_seated_pose_right_arm


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


initialization_dict ={	'ChildName' : "None",
						'MoveHandSpeed'	: 0.1,
						'ReadDelay'	: 0.1,
						'ReadSpeed'	: "70",
						'ReactionDelay' : 1,
						'HandPoint' : False
					 }
					 

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

speakingText= {	"Hello , My name is Joaquim , what is your name?",
	            "Hello Sierra , Nice to meet you",
	            "I really like mathematics , what about you?",
	            "Wow , that's great",
	            "Oh , maybe you like it more after we practice together",
	            "Do you know what is an abacus?",
	            "Awesome , then let's do some practice",
	            "Don't worry , I also don't know much"}

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


def TaskAssignment(groupNumber, orderCounter, prodSeries):

	global finishProTask
	global finishCompTask
	global yekan
	global dahgan
	global sadgan

	orderCounter += 1
	prodSeries += 1

	if groupNumber == "AE":
		for j in range(1,4):
			prodSeries = j
			#ComprehensionTask("ambiguous")
			ProductionTask(prodSeries)
			print ("finishProTask: " + str(finishProTask))
			if finishProTask:
				if j == 1:
					ComprehensionTask("ambiguous")
				#elif j == 2:
				#	ComprehensionTask("egocentric")
				#	if finishCompTask:
				#		orderCounter = j+1
				else:
					GoToFinishLine(2, "kiddo")
					#if finishCompTask:
						#orderCounter = j+1

	if groupNumber == "AD":
		for j in range(1,4):
			prodSeries = j
			ProductionTask(prodSeries)
			print ("finishProTask: " + str(finishProTask))
			if finishProTask:
				if j == 1:
					ComprehensionTask("ambiguous")
				elif j == 2:
					ComprehensionTask("addressee")
					if finishCompTask:
						orderCounter = j+1
				else:
					GoToFinishLine(2, "kiddo")
					if finishCompTask:
						orderCounter = j+1


	if groupNumber == "EA":
		for j in range(1,4):
			prodSeries = j
			ProductionTask(prodSeries)
			print ("finishProTask: " + str(finishProTask))
			if finishProTask:
				if j == 1:
					ComprehensionTask("egocentric")
				elif j == 2:
					ComprehensionTask("ambiguous")
					if finishCompTask:
						orderCounter = j+1
				else:
					GoToFinishLine(2, "kiddo")
					if finishCompTask:
						orderCounter = j+1

	if groupNumber == "DA":
		for j in range(1,4):
			prodSeries = j
			ProductionTask(prodSeries)
			print ("finishProTask: " + str(finishProTask))
			if finishProTask:
				if j == 1:
					ComprehensionTask("addressee")
				elif j == 2:
					ComprehensionTask("ambiguous")
					if finishCompTask:
						orderCounter = j+1
				else:
					GoToFinishLine(2, "kiddo")
					if finishCompTask:
						orderCounter = j+1


def print_row(data):
	print ("-------------------------------------------------")
	print (data)

def ProductionTask(prodSeries):
	global finishProTask
	global proOrder
	#global prodSeries
	proOrder = 1
	
	production_numbers = rospy.get_param('~production_numbers_series1')
	numbersArray = production_numbers.splitlines()
	print numbersArray

	# Seperate the numbers in each line
	elements = []
	for line in numbersArray:
		elements.append(re.split('\W+,', line))
	elements = elements[2:]

	print ("elements: ")
	print elements
	print "prodSeries"
	print prodSeries


	#Bead4_location = rospy.Subscriber('/abacus/row2/bead4/xchange', String, print_location)
	#print ("Bead4_location")
	#print (Bead4_location)
	#row1_count = rospy.Subscriber('/abacus/row2/count', String, print_row)


	if prodSeries == 1:
		productionBatch = elements[(prodSeries - 1)*5: (prodSeries)*5]
		print ("productionBatch")
		print (productionBatch)
		

	if prodSeries == 2:
		productionBatch = elements[(prodSeries - 1)*5 + 1 : (prodSeries)*5 + 1]
		print ("productionBatch")
		print (productionBatch)


	if prodSeries == 3:
		productionBatch = elements[(prodSeries - 1)*5 + 2: (prodSeries)*5 + 2]
		print ("productionBatch")
		print (productionBatch)


	if prodSeries == 4:
		productionBatch = elements[(prodSeries - 1)*5 + 3: (prodSeries)*5 + 3]
		print ("productionBatch")
		print (productionBatch)


	
	elements = []
	for line in numbersArray:
		elements.append(re.split('\W+,', line))
	print elements
	print elements[2]



	TrialBehaviour()
	robot_answers_dict[1] = True
	robot_answers_dict[2] = True
	for k in range(5):
		ProductionInteractionScenario(prodSeries, proOrder, productionBatch)
		proOrder +=  1

	for key in sorted(confusion_matrix_dict.iterkeys()):
	    print "%s: %s" % (key, confusion_matrix_dict[key])
	    confusin_mat_prod_pub.publish( key + "  :"+ confusion_matrix_dict[key])


	finishProTask = True


def TrialBehaviour():
	TrialNumber = 0
	Response = True
	print robot_answers_dict
	
	TrialNumber += 1
	order = random.sample(range(2,5), 2)
	ordered_order = sorted(order, key=int)
	print ordered_order
	robot_answers_dict[order[0]] = False
	robot_answers_dict[order[1]] = False
	print robot_answers_dict

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
	
	if value < 0:
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
	 	#position1 = [0.3 + row * 0.2, XA - Slice * step, 0.1]
	 	position1 = [0.3 + (row - 1) * 0.2, XA - Slice * step, 0.1]
	 	position2 = [0.7, 0, 0.1]
		position3 = [0.7, -0.2, 0.1]
	elif direction == "Right":
		#position1 = [0.3 + row * 0.2, XB + Slice * step + Const, 0.1]
		position1 = [0.3 + (row - 1) * 0.2, XB + Slice * step + Const, 0.1]
		position2 = [0.7, 0, 0.1]
		position3 = [0.7, 0.2, 0.1]

	curPose = motionProxy.getPosition("RHand", frame, useSensorValues)
	print ("dirrection, step, turn, row, stay", direction, step, turn, row, stay)
	print "position----------------------------", position1
	# Tracker
	tracker.post.lookAt(position1, frame, maxSpeedHead, useWholeBody)
	tracker.pointAt("RArm", position1, frame, maxSpeedHead)

	# Robot goes back to resting position
	if step == turn and not stay:
		postureProxy.goToPosture("Crouch", 0.3)




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
	#tracking_enabled = True
	#faceProxy.post.enableTracking(tracking_enabled)

	# Then, start tracker
	tracker.track(targetName)


def faceTrackingEnded():
	""" Robot stops to track the users face and go into resting mode after certain keys are pressed

	"""
	tracking_enabled = False
	faceProxy.enableTracking(tracking_enabled)
	#tracker.stopTracker()
	#tracker.unregisterAllTargets()
	#motionProxy.rest()


def LookAtTheTablet(pitch_angle=0, yaw_angle=0):
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
		print "on"
	elif Switch == False:
		faceTrackingEnded()
		print "off"

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


def IntroduceNao(numOfStart, childName):
	"""
	Nao starts introducing itself
	"""
	global robot_state_pub
	global blinkThread 

	# Initialization of the robot behavior
	#----------------------------------------------------------------
	"""postureProxy.goToPosture("Crouch", 0.4)
	communicate.setLanguage('English')
	robot_state_pub.publish("Introduce Nao Started")
	turn_on_eye()
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	blinkingModeON("ON")"""

	if numOfStart == "1":
		tracking_enabled = True
		#faceProxy.enableTracking(tracking_enabled)

	if numOfStart == "2":
		script =   ("\\rspd=90\\ Hello" + childName + " \\pau=1000\\ "
					"\\rspd=90\\ My name is joaquim\\pau=1000\\"
					"\\rspd=90\\ I love math and I want to learn how to use an abacos\\pau=700\\"
					"\\rspd=80\\ what about you?\\pau=3000\\"
					"\\rspd=90\\ My math level is similar to you\\pau=500\\"
					"\\rspd=90\\ and sometimes I might make mistakes\\pau=900\\"
					"\\rspd=80\\ I would be glad if you help me \\pau=1000\\"
					"\\rspd=80\\ Should we start now?\\pau=500\\")
	else:
		script = ("\\rspd=90\\ ")
	
	#communicate.say(script)

	print ("IntroduceNao")

	robot_instruction_pub.publish(script)
	time.sleep(0.5)
	robot_state_pub.publish("Introduce Nao Ended")


"""
def IntroduceNao(numOfStart,childName):
	if numOfStart == "1":
		print ("Hello " + childName + "\\pau=700\\ Welcome back \\pau=500\\")
		#communicate.say("\\rspd=90\\ Hello everyone \\pau=700\\ My name is Nao")
		postureProxy.goToPosture("Crouch", 0.2)
"""

def InitializeNao():
	"""
	"""

	# Initialization of the robot behavior
	#----------------------------------------------------------------
	postureProxy.goToPosture("Crouch", 0.4)
	communicate.setLanguage('English')
	robot_state_pub.publish("Nao Initialized")
	turn_on_eye()
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	blinkingModeON("ON")

	tracking_enabled = True
	faceTrackingEnded()
	#faceProxy.enableTracking(tracking_enabled)




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

def RowDecision(value, row):
	"""

	"""
	
	# Initialization
	# -----------------------------------
	nbBead = 10
	turn = 5
	stay = False
	global reset_publish


	# Subscribe to the max diatance and the size of the beads recieved from the virtual abacus side
	# ---------------------------------------------------------------------------------------------
	rospy.Subscriber('/abacus/bead/radius', String, PrintRadius)
	rospy.Subscriber('/abacus/row/length', String, PrintLength)
	beadRadius = basic_data_dict['radius']
	maxRowLength = basic_data_dict['length']
	print "in RowDecision"
	print value

	

	if value < nbBead:
		stay = False
		MoveBeadPublisher(value, maxRowLength, beadRadius, turn, stay)



	if value > nbBead:

		if value % 10 == 0:
			if row == "ROW1":
				stay = True
				# Move 10 beads in first row
				MoveBeadPublisher(10, maxRowLength, beadRadius, turn, stay)
				time.sleep(0.5)

				# Move the 10 beads back to initial position in first row
				PointAtTheTablet("Left", 1, 2, 1, stay, -0.01)
				reset_publish.publish("1")
				PointAtTheTablet("Left", 2, 2, 1, stay, -0.01)
				time.sleep(0.5)

				# Move 1 bead in the second row
				stay = False
				MoveBeadPublisherRow2(value/10, maxRowLength, beadRadius, turn, stay, 0, 1)

			if row == "ROW2" or row == "":
				stay = False
				MoveBeadPublisherRow2(value/10 , maxRowLength, beadRadius, turn, stay, 0, 1)

		else:

			#belowTen = nbBead - value
			stay = True

			
			# Move the beads back to initial position in the first row
			PointAtTheTablet("Left", 1, 2, 1, stay, -0.01)
			reset_publish.publish("1")
			PointAtTheTablet("Left", 2, 2, 1, stay, -0.01)
			time.sleep(0.5)
			
			# Move one bead from the second row to count the 10
			MoveBeadPublisherRow2(1, maxRowLength, beadRadius, turn, stay, 0, 1)
			AboveTen = value - nbBead
			time.sleep(1.5)

			# Move the rest of the beads in the first row to find the final answer
			stay = False
			MoveBeadPublisher(AboveTen, maxRowLength, beadRadius, turn, stay)



def MoveBeads(data):
	"""
	The function for adding the selected number to the instructions requiring the number being specified
	"""

	# GLOABALS
	# ----------------------------------
	global instructionText

	# Initialization
	# -----------------------------------
	speakingText = "\\rspd=80\\"
	elements = []
	i = 0
	row = ""
	LookAtTheChild(False)
	# Processing 
	# ----------------------------------------
	movingValue = data.data

	status = instructionText[0]
	instructionText = instructionText[1:]
	print "In MoveBeads"
	print instructionText, movingValue


	# Loop to replace X with the specified number 
	# --------------------------------------------
	for text in instructionText:

		if text == " X":
			instructionText[i] = movingValue

		if text == " ROW1":
			row = "ROW1"
			instructionText[i] = ""
			
		if text == " ROW2":
			row = "ROW2"
			instructionText[i] = ""

		speakingText += instructionText[i] + "\\pau=500\\"
		i += 1


	# Decision-making based on the instruction type
	# ---------------------------------------------
	if status == " INSTRUCTION":
		communicate.say(speakingText)

	elif status == " ACTION":
		LookAtTheChild(False)
		# Turning on and off the breathing mode or it will affect the pointing behavior of the robot
		communicate.say(speakingText)
		motionProxy.setBreathEnabled('Arms', False)
		RowDecision( int(movingValue), row )
		motionProxy.setBreathEnabled('Arms', True)

	elif status == " ADD":
		LookAtTheChild(False)
		communicate.say(speakingText)
		motionProxy.setBreathEnabled('Arms', False)
		RowDecision( int(movingValue), row )
		motionProxy.setBreathEnabled('Arms', True)


	instructionText = []



def ButtonPressed(data, childName):
	"""
	The decision-making function for each time a button is pressed from control panel
	"""

	# GLOBALS
	global instructionText
	global reset_publish

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
	instructionText = instructionPart
	print "In ButtonPressed"
	print instructionText


	# Decision-making based on which column the  instruction is coming from
	# ----------------------------------------------------------------------
	if activeColumn == "Introduction":

		if elements[0][1] == " Start the robot":
			InitializeNao()

		else:
			LookAtTheChild(True)
			RobotSpeaking(instructionText, childName)

	elif activeColumn == "Practice":

		# Decision-making based on the instruction type
		# ----------------------------------------------
		if status == " RESPONSE":
			LookAtTheChild(False)
			RobotSpeaking(instructionText[1:], childName)

		elif status == " RESET":
			LookAtTheChild(False)
			RobotSpeaking(instructionText[1:], childName)
			motionProxy.setBreathEnabled('Arms', False)

			PointAtTheTablet("Left", 1, 2, 1, True, -0.01)
			reset_publish.publish("1")
			PointAtTheTablet("Left", 2, 2, 1, False, -0.01)
			
			motionProxy.setBreathEnabled('Arms', True)

		elif status == " INSTRUCTION" or status == " ACTION":
			LookAtTheChild(False)

			rospy.Subscriber('/control/button/number', String, MoveBeads, queue_size=1)

	#LookAtTheChild(True)

	#instructionText = []



	


def main():

	# Initiate the node that does the main computing
	rospy.init_node('main_scenario')

	# General Initiation
	##########################################################################
	global restingEnabled 
	restingEnabled = False
	
	global space
	space      = motion.FRAME_ROBOT

	global RobotConnected
	RobotConnected = True


	# Inititate the publishers
	##########################################################################
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

	global reset_publish
	reset_publish = rospy.Publisher('reset/order', String, queue_size=1)

	global position_publish
	"""for i in range(0, 10):
		position_publish = rospy.Publisher('abacus/row0/bead'+ str(i) + '/setX', String, queue_size=1)
		position_publish = rospy.Publisher('abacus/row1/bead'+ str(i) + '/setX', String, queue_size=1)
		position_publish = rospy.Publisher('abacus/row2/bead'+ str(i) + '/setX', String, queue_size=1)
	"""

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

	# Proxies
	###################################################################################
	global communicate
	communicate = ALProxy("ALTextToSpeech", nao_IP, nao_port)
	communicate.setLanguage('Portuguese')
	#communicate.setVolume(1)

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

	#subscriber = memory.subscribeToEvent("ALTextToSpeech/TextDone", "SpeechCondition", "onTextDone")
	groupNumber = "AX"
	orderCounter = 0
	productionSeries = 0

	global maxSpeedHead

	initialiazation_file = rospy.get_param('~initialization_file')
	#with open('./TextFiles/customized_instruction.txt') as f:
	instructionText = initialiazation_file

	#with open('./TextFiles/customized_instruction.txt') as f:
		#instructionText = f.read()
	instructionArray = initialiazation_file.splitlines()

	numOfStart = instructionArray[1]
	initialization_dict["ChildName"] = instructionArray[3]
	initialization_dict["groupNumber"] = instructionArray[5]
	initialization_dict["childCondition"] = instructionArray[7]
	initialization_dict["robotCondition"] = instructionArray[9]
	initialization_dict["childTaskNumber"] = instructionArray[11]
	initialization_dict["robotTaskNumber"] = instructionArray[13]
	initialization_dict["WhichPage"] = instructionArray[15]
	initialization_dict["CorrectionState"] = instructionArray[17]
	initialization_dict["BookSelection"] = instructionArray[19]


	# Initialize the log file
	logFile = logger()
	time.sleep(1)
	robot_instruction_pub.publish(instructionText)
	time.sleep(1)

	childName = initialization_dict["ChildName"]
	child_name_pub.publish(childName)
	correction_state_pub.publish("correct")

	facesize = 0.1

	#postureProxy.goToPosture("Crouch", 0.4)
	#tracking_enabled = True
	#faceProxy.enableTracking(tracking_enabled)
	#communicate.say("\\rspd=80\\ Should we start now?\\pau=500\\")

	# The function to introduce Nao and stop the face tracking 
	#IntroduceNao(numOfStart, childName)

	#motionProxy.setBreathEnabled('Arms', False)

	# Something that can be stated in the initializationFile

	##### Start the assignment of tasks
	#TaskAssignment(groupNumber, orderCounter, productionSeries)
	childName = ""

	rospy.Subscriber('/control/button/content', String, ButtonPressed, childName, queue_size=1)

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

