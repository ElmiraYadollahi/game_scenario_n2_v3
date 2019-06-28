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


def TaskAssignment(groupCode, orderCounter, prodSeries):

	global finishProTask
	global finishCompTask
	global yekan
	global dahgan
	global sadgan

	orderCounter += 1
	prodSeries += 1

	if groupCode == "AE":
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

	if groupCode == "AD":
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


	if groupCode == "EA":
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

	if groupCode == "DA":
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
	 	position1 = [0.3 + row * 0.2, XA - Slice * step, 0.1]
	 	position2 = [0.7, 0, 0.1]
		position3 = [0.7, -0.2, 0.1]
	elif direction == "Right":
		position1 = [0.3 + row * 0.2, XB + Slice * step + Const, 0.1]
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

def DecideMovement(msg, dict_key):
	global whilecounter
	if whilecounter == 0:

		
		print "whilecounter" 
		print whilecounter

		elements = []
		content = msg.data
		elements.append( re.split('\W+,', content) )

		print "subscribed"

		status = elements[0][0]
		dict_fill = elements [0][1:]

		print status
		print dict_fill

		if status == "WAIT":
			robot_move_dict [dict_key] = "WAIT"
			time.sleep(1)
			print "in wait status"
			whilecounter += 1

		if status == "ACTION":
			robot_move_dict [dict_key] = "ACTION"
			time.sleep(1)
			print "in action status"
			whilecounter += 1

def PrintButtonCorrect (data):
	print (data)

def PrintButtonWrong (data):
	print (data)

def RobotUtterances(interactionPoint):
	"""

	"""
	randomNum = random.randint(1,3)

	if randomNum == 1:
		firstWords = "\\rspd=80\\ I'm ready"
		waitWords = "\\rspd=80\\ Sorry, but I don't know what to do"
		secondWords = ""
	if randomNum == 2:
		firstWords = "\\rspd=80\\ Let's go"
		waitWords = "Sorry, but I don't exatly understand"
		secondWords = ""
	if randomNum == 3:
		firstWords = "\\rspd=80\\ I am ready"
		waitWords = "\\rspd=80\\ wait, you mean I should move the beads?"
		secondWords = ""

	if interactionPoint == "First Number":
		communicate.say(firstWords)

	elif interactionPoint == "WAIT":
		communicate.say(waitWords)

	elif interactionPoint == "Second Number":
		communicate.say(secondWords)


def ButtonPressed(data):
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
	if activeColumn == "Child-Turn":

		if status == " WAIT":
			LookAtTheChild(False)
			RobotSpeaking(instructionText[1:])
			waitCounter += 1

		elif status == " ACTION":
			LookAtTheChild(False)
			RobotSpeaking(instructionText[1:])
			ActioCounter += 1

	elif activeColumn == "Robot-Turn":

		# Decision-making based on the instruction type
		# ----------------------------------------------
		if status == " RESPONSE":
			LookAtTheChild(False)
			RobotSpeaking(instructionText[1:])

		elif status == " RESET":
			LookAtTheChild(False)
			RobotSpeaking(instructionText[1:])
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

def ProductionInteractionScenario(prodSeries, proOrder, productionBatch):
	"""	Robot looks at the child when the child talks
		robot looks at the abacus
		and when the child asks what is the number? 
		robot answers based on the reading from the file
	"""
	
	# Initialization
	nbBead = 10
	turn = 3
	stay = False
	whileCondition = True
	
	#robot_move_dict [dict_key] = "WAIT"

	
	
	# CALCULATION
	#######################################################################
	reset_publish = rospy.Publisher('reset/order', String, queue_size=1)
	reset_publish.publish("1")

	# The numbers that the child has in their card
	#-------------------------------------
	currenetElements = [int(i) for i in productionBatch[proOrder-1]]
	print "currenetElements"
	print currenetElements


	dict_key = "P " + str(prodSeries) + "&" + str(proOrder) + ": " + str(currenetElements)
	robot_move_dict [dict_key] = "DO NOTHING"

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
	reset_publish.publish("0")
	print ("reset")
	RobotUtterances("First Number")
	

	
	# Child tells the robot to show a number
	# child uses a card to give instruction
	# robot knows that number based on the production_series data set
	# the numbers for each trial is stored in currentElements 
	#------------------------------------
	#time.sleep(5)
	firstValue = currenetElements[0]

	# robot acts like it has heard the child, so now it looks at the tablet 
	# and shows the number requested
	# it also moves its hand to point at it 
	LookAtTheChild(False)
	time.sleep(1)

	# Wait for control decision message
	# ------------------------------------------
	

	# Moving beads operation
	# ------------------------------------------
	print("robot looking at the tablet")
	LookAtTheTablet()
	#MoveBeadPublisher(firstValue, maxRowLength, beadRadius, turn, stay)

	# robot looks at the child again
	#------------------------------------------
	LookAtTheChild(True)


	#time.sleep(10)
	global whilecounter
	whilecounter = 0
	#while whileCondition:
	#print "inwhile"
	#print whilecounter
	print robot_move_dict [dict_key]
	#if robot_move_dict [dict_key] == "DO NOTHING":
	#	whilecounter = 0

	#if whilecounter == 0:
		#rospy.Subscriber('/control/button/content', String, ButtonPressed, queue_size=1)
	rospy.Subscriber('/control/childturn/content', String, DecideMovement, dict_key)

	if robot_move_dict [dict_key] == "WAIT":
		RobotUtterances("WAIT")
		#whilecounter = 0
		print "inwait"
		print whilecounter
		whilecounter = 0


	elif robot_move_dict [dict_key] == "ACTION":

		secondValue = currenetElements[1]

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




		# robot_answers_dict is a dictionary that provide a random plan of 3 correct and 2 wrong answers for the robot
		# the robot answers based on what is on this dict
		#---------------------------------------------------
		if robot_answers_dict[proOrder]:
			# robot moves correct number of beads
			print ( secondValue)
			XsecondValue = secondValue
			answerValue = firstValue + secondValue
			print "correct mode"
			print ("Now the answer is " + str(answerValue))
			robot_response = True

		elif not (robot_answers_dict[proOrder]):
			print (wrongSecondValue)
			XsecondValue = wrongSecondValue
			answerValue = firstValue + wrongSecondValue
			print "wrong mode"
			print ("Now the answer is " + str(answerValue))
			robot_response = False
		
		
		
		# ACTION
		###########################################################################
		# robot moves the second value based on the "order of trial" which means right or worng answers
		LookAtTheChild(False)
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

		time.sleep(4)

		# At this point the robot tells the final answer to the child
		LookAtTheChild(True)
		communicate.say ("The answer is " + str(answerValue))
		communicate.say ("Can you tell me if my answer was correct?")
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


		confusion_matrix_dict ["P " + str(prodSeries) + "&" + str(proOrder) + ": " + str(currenetElements)] = answerCondition

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
		if child_response:
			communicate.say ("I am so happy?")
		elif not child_response:
			communicate.say ("I am sorry, I will try to be more careful")


		if proOrder == 5:
			communicate.say ("I sloved a lot of questions, now it is your turn to solve my questions")
		else:
			communicate.say ("Let's solve another question")


		whileCondition = False

	

def ComprehensionInteractionScenario():
	h = 0 




def ComprehensionTask(compTypeSelection):
	"""

	"""

	# Initialization
	global finishCompTask
	elements = []
	compOrder = 1

	compType = compType_dict[compTypeSelection]

	# CALCULATION
	#######################################################################################
	comprehension_numbers = rospy.get_param('~comprehension_numbers_series1')
	numbersArray = comprehension_numbers.splitlines()

	for line in numbersArray:
		elements.append(re.split('\W+,', line))
	
	elements = elements[2:]


	# CONDITIONS
	#####################################################################################
	if compType == 1:
		comprehensionBatch = elements[(compType - 1)*5: (compType)*5]
		time.sleep(1)

		for k in range (5):
			time.sleep(1)
			AmbigiousBehaviour(comprehensionBatch, k)

	elif compType == 2:
		comprehensionBatch = elements[(compType - 1)*5 + 1: (compType)*5 + 1]
		time.sleep(1)

		for k in range (5):
			time.sleep(1)
			EgocentricBehaviour(comprehensionBatch, k)

	elif compType == 3:
		comprehensionBatch = elements[(compType - 1)*5 + 2: (compType)*5 + 2]
		time.sleep(1)

		for k in range (5):
			time.sleep(1)
			AddresseeBehaviour(comprehensionBatch, k)

	finishCompTask = True


def AmbigiousBehaviour(comprehensionBatch, compOrder):
	"""
	Ambiguous Behaniour

	"""

	# Initializations
	global confusion_mat_comp_pub	
	global yekan
	global dahgan
	global sadgan

	# The numbers that the robot asks to the child
	#--------------------------------------------------- 
	currenetElements = [int(i) for i in comprehensionBatch[compOrder]]

	firstValue = currenetElements[0]
	secondValue = currenetElements[1]
	answerValue = currenetElements[2]

	# Interaction
	#-----------------------------------------------------
	print (" First, show me " + str(firstValue) + " on the abacus")
	communicate.say (" First, show me " + str(firstValue) + " on the abacus")
	

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

	print ("\\rspd=90\\ can you move " + str(secondValue) + " beads to the right?")
	communicate.say (" First, show me " + str(firstValue) + " on the abacus")
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


	#confusion_matrix_dict ["C " + "ambiguous" + "&" + str(compOrder) + ": " + str(currenetElements)] = answerCondition



def EgocentricBehaviour(comprehensionBatch, compOrder):
	#communicate.post.say("\\rspd=90\\ can you move two beads to my right?")
	currenetElements = [int(i) for i in comprehensionBatch[compOrder]]
	print ("currenetElements")
	print (currenetElements)


	print (" First, show me " + str(currenetElements[0]) + " on the abacus")
	print ("\\rspd=90\\ can you move " + str(currenetElements[1]) + " beads to my right?")
	print (" Good Job")
	print ("what is the answer?")


def AddresseeBehaviour(comprehensionBatch, compOrder):
	#communicate.post.say("\\rspd=90\\ can you move two beads to your right?")
	currenetElements = [int(i) for i in comprehensionBatch[compOrder]]
	print ("currenetElements")
	print (currenetElements)


	print (" First, show me " + str(currenetElements[0]) + " on the abacus")
	print ("\\rspd=90\\ can you move " + str(currenetElements[1]) + " beads to your right?")
	print (" Good Job")
	print ("what is the answer?")



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


def LookAtTheTablet(pitch_angle=0, yaw_angle=0):
	""" Move the robot's head to look at the tablet

	"""
	global fractionMaxSpeed
	motionProxy.setStiffnesses("Head", 1.0)
	pitch_angle = 0.3
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



def ChildTurnInteractionScenario():
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
	currenetElements = [int(i) for i in productionBatch[proOrder-1]]
	print "currenetElements"
	print currenetElements


	#dict_key = "P " + str(prodSeries) + "&" + str(proOrder) + ": " + str(currenetElements)
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
	reset_publish.publish("0")
	print ("reset")
	RobotUtterances("First Number")
	

	
	# Child tells the robot to show a number
	# child uses a card to give instruction
	# robot knows that number based on the production_series data set
	# the numbers for each trial is stored in currentElements 
	#------------------------------------
	#time.sleep(5)
	firstValue = currenetElements[0]

	# robot acts like it has heard the child, so now it looks at the tablet 
	# and shows the number requested
	# it also moves its hand to point at it 
	LookAtTheChild(False)
	time.sleep(1)

	# Wait for control decision message
	# ------------------------------------------
	

	# Moving beads operation
	# ------------------------------------------
	print("robot looking at the tablet")
	LookAtTheTablet()
	#MoveBeadPublisher(firstValue, maxRowLength, beadRadius, turn, stay)

	# robot looks at the child again
	#------------------------------------------
	LookAtTheChild(True)


def DecisionMakingThread(data):
	"""
	This thread receives the instructions from the other side
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
	print "activeColumn, status, In ButtonPressed"
	print activeColumn, status, instructionText

	initialization_dict["groupCode"]
	dict_key = 	"P" + str(initialization_dict["groupCode"]) + \
				"C" + str(initialization_dict["childTurnCondition"]) + \
				"T" + str(initialization_dict["childTaskNumber"])


	childTurnCondition = initialization_dict["childTurnCondition"]
	if status == "START":
		current_action_dict [dict_key] = "START"
		currenetElements = [int(i) for i in productionBatch[childTurnCondition - 1]]
		ChildTurnInteractionScenario()






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
	nao_IP = '192.168.1.64'
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
	communicate.setVolume(0.4)


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
	initialization_dict["childTurnCondition"] = instructionArray[7]
	initialization_dict["robotTurnCondition"] = instructionArray[9]
	initialization_dict["childTaskNumber"] = instructionArray[11]
	initialization_dict["robotTaskNumber"] = instructionArray[13]
	initialization_dict["WhichPage"] = instructionArray[15]
	initialization_dict["CorrectionState"] = instructionArray[17]
	initialization_dict["BookSelection"] = instructionArray[19]


	# Initialize the log file
	logFile = logger()
	time.sleep(5)
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
