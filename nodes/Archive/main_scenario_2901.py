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
from memory.msg import Animation

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

""" It is a dictionary for the states of the tag:
	False: 						means the story have not be read yet, it will go into reading with wrong tag 
	"Waiting for Red-card":		means the robot is waiting to recieve a green card
	"Waiting for Yellow-card":	robot repeats reading the text
	"Waiting for Green-card":	robot reacts happily
	"correcting Mode": 			means the story have been read and the robot have recieved a red card
	"Corrected Mode":		
	"Correct Happy Mode"
	"Wrong Happy Mode"	
	True:						means the story have been read and the robot have recieved a green card

"""
pairs_dict = {	'[0, 1]' : False,
				'[2, 3]' : False,
				'[4, 5]' : False,
				'[6, 7]' : False,
				'[8, 9]' : False,
				'[10, 11]' : False,
				'[12, 13, 14, 15]' : False,
				'[12, 13]' : False,
				'[ 14, 15]' : False,
				'[16, 17]' : False,
				'[18, 19, 20, 21]' : False,
				'[18, 19]' : False,
				'[ 20, 21]' : False,
				'[22, 23]' : False,
				'[24, 25, 26, 27]' : False,
				'[24, 25]' : False,
				'[ 26, 27]' : False,
				'[28, 29]' : False,
				'[30, 31, 32, 33]' : False,


				'[30, 31]' : False,
				'[32, 33]' : False,
				'[34, 35, 36, 37]' : False,
				'[34, 35]' : False,
				'[ 36, 37]' : False,

				'[41, 42, 43, 44]' : False,
				'[41, 42]' : False,
				'[ 43, 44]' : False,

				'[45, 46, 47, 48]' : False,
				'[45, 46]' : False,
				'[ 47, 48]' : False,				

				'[49, 50, 51, 52]' : False,
				'[49, 50]' : False,
				'[ 51, 52]' : False,

				'[53, 54, 55, 56]' : False,
				'[53, 54]' : False,
				'[ 55, 56]' : False,

				'[57, 58, 59, 60]' : False,
				'[57, 58]' : False,
				'[ 59, 60]' : False,

				'[61, 62, 63, 64]' : False,
				'[61, 62]' : False,
				'[ 63, 64]' : False,

				'[65, 66, 67, 68]' : False,
				'[65, 66]' : False,
				'[ 67, 68]' : False,

				'[69, 70, 71, 72]' : False,
				'[69, 70]' : False,
				'[ 71, 72]' : False,

				'[73, 74, 75, 76]' : False,
				'[73, 74]' : False,
				'[ 75, 76]' : False,

				'[77, 78, 79, 80]' : False,
				'[77, 78]' : False,
				'[ 79, 80]' : False,

				'[81, 82]' : False,
				'[83, 84]' : False,

				'[85, 86]' : False,



				'[90, 91]' : False,
				'[92, 93]' : False,

				'[94, 95, 96, 97]' : False,
				'[94, 95]' : False,
				'[ 96, 97]' : False,

				'[98, 99, 100, 101]' : False,
				'[98, 99]' : False,
				'[ 100, 101]' : False,

				'[102, 103, 104, 105]' : False,
				'[102, 103]' : False,
				'[ 104, 105]' : False,

				'[106, 107, 108, 109]' : False,
				'[106, 107]' : False,
				'[ 108, 109]' : False,

				'[110, 111, 112, 113]' : False,
				'[110, 111]' : False,
				'[ 112, 113]' : False,

				'[114, 115, 116, 117]' : False,
				'[114, 115]' : False,
				'[ 116, 117]' : False,

				'[118, 119, 120, 121]' : False,
				'[118, 119]' : False,
				'[ 120, 121]' : False,

				'[122, 123, 124, 125]' : False,
				'[122, 123]' : False,
				'[ 124, 125]' : False,

				'[126, 127, 128, 129]' : False,
				'[126, 127]' : False,
				'[ 128, 129]' : False,

				'[130, 131, 132, 133]' : False,
				'[130, 131]' : False,
				'[ 132, 133]' : False,

				'[134, 135, 136, 137]' : False,
				'[134, 135]' : False,
				'[ 136, 137]' : False,

				'[138, 139]' : False,





				'[62, 63]' : False,
				'[64, 65]' : False,
				'[66, 67, 68, 69]' : False,
				'[66, 67]' : False,
				'[ 68, 69]' : False,
				'[70, 71, 72, 73]' : False,
				'[70, 71]' : False,
				'[ 72, 73]' : False,
				'[74, 75, 76, 77]' : False,
				'[74, 75]' : False,
				'[ 76, 77]' : False,
				'[78, 79]' : False,



				'[250, 251]' : False,
				'[252, 253]' : False,
				'[200, 201, 202, 203]' : False,
				'[200, 201]' : False,
				'[ 202, 203]' : False,
				'[204, 205, 206, 207]' : False,
				'[204, 205]' : False,
				'[ 206, 207]' : False,
				'[208, 209, 210, 211]' : False,
				'[208, 209]' : False,
				'[ 210, 211]' : False,
				'[212, 213, 214, 215]' : False,
				'[212, 213]' : False,
				'[ 214, 215]' : False,
				'[216, 217, 218, 219]' : False,
				'[216, 217]' : False,
				'[ 218, 219]' : False,
				'[220, 221, 222, 223]' : False,
				'[220, 221]' : False,
				'[ 222, 223]' : False,
				'[224, 225]' : False,
				'[226, 227, 228, 229]' : False,
				'[226, 227]' : False,
				'[ 228, 229]' : False,
				'[230, 231, 232, 233]' : False,
				'[230, 231]' : False,
				'[ 232, 233]' : False,
				'[234, 235, 236, 237]' : False,
				'[234, 235]' : False,
				'[ 236, 237]' : False,
				
				'[256, 257]' : False,
				'[258, 259]' : False,
				'[260, 261]' : False,
				'[262, 263]' : False,
				'[264, 265]' : False,
				'[266, 267]' : False,
				'[268, 269]' : False,
				'[270, 271]' : False,
				'[272, 273]' : False,
				'[274, 275]' : False,
				'[276, 277]' : False,
				'[278, 279]' : False
			 }


communicate_dict ={	'[200, 201]': 1, 
				'[210, 211]': 2,
				'[220, 221]': 3,
				'[230, 231]': 4,
				'[240, 241]': 5,
				'[2, 3]': 7
			}

taskLevel_dict ={	'CurrLevel'		: "TaskNONE",
					'PrevLevel'		: "TaskNONE",
					'LevelONE'		: 1,
					'LevelTWO'		: 1,
					'LevelTHREE'	: 1,
					'LevelTHREE'	: 0
				}


initialization_dict ={	'ChildName' : "None",
						'MoveHandSpeed'	: 0.1,
						'ReadDelay'	: 0.1,
						'ReadSpeed'	: "70",
						'ReactionDelay' : 1,
						'HandPoint' : False
					 }


alert_dict ={	'ChildName' : "None",
						'MoveHandSpeed'	: 0.1,
						'ReadDelay'	: 0.1,
						'ReadSpeed'	: "70",
						'ReactionDelay' : 1,
						'HandPoint' : False,
						'WhichPage' : 1,
						'CorrectionState' : False,
						'BookSelection' : False,
					 }


card_id = { 38, 39, 40}


instruction_tags_dict = {	'=LineNum',
							'=wordNum',
							'=MistakeNum',
							'=NextLine',
							'=Skip',
							'=L1',
							'=L2',
							'=L3',
							'=L4',
							'=L5',
							'=L6',
							'=L7',
							'=L8',
							'=L9',
							'=L10',
							'=L11',
							'=B1',
							'=B1',
							'=B2',
							'=B3',
							'=B4',
							'=B5',
							'=B6',
							'=B7',
							'=B8',
							'=B9',
							'=B10',
							'=B11',
						}


motionProxy_states = { "low": [1, 2, 3, 4, 5],
				  	"medium": [6, 7, 8, 9, 10],
				  	"high": [11, 12, 13, 14, 15] }


mistake_level_states = { "low": [1],
						 "medium": [3],
						 "high": [5]}

story_selection_state = {}

stop_state = { "StopReading": False,
				"FinishReading": False}

""" Recieve the key stroke from key publisher node
	[1]: tell the robot made a mistake in reading the story
	[2]: tell the robot to choose a new story and repeat the process
	[3]: tell the robot to stop the tracker and go to resting mode

"""
KEY_MAPPING = { 'f': [2], 
				'g': [2], 
				'h': [2],
				'b': [3],
				'n': [3],
				'm': [3],
				'e': [4],
				'r': [4],
				't': [4],
				's': [1]  
				}




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
global ARTag
global REDButtonAllert
REDButtonAllert = False
global YELLOWButtonAllert
YELLOWButtonAllert = False
global NumOfGreenCard
NumOfGreenCard = 0
global FinishSign
FinishSign = False
global lastBreakPoint 
lastBreakPoint = [0, 0]
global effector
effector = "RArm"
global once
once = 0
#global mistakeExist
#mistakeExist = True
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

confusion_matrix_dict = { 'equation'	: "condition"
						
						}


compType_dict = {	'ambiguous'		: 1,
					'egocentric'	: 2,
					'addressee'		: 3,
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








def TaskAssignment(groupNumber, orderCounter, prodSeries):
	global finishProTask
	global finishCompTask
	global yekkan
	global dahgan
	global sadgan
	orderCounter += 1
	prodSeries += 1
	#rospy.Subscriber('/abacus/row' + str(0) + '/count', String, PrintRow1)
	#rospy.Subscriber('/abacus/row' + str(1) + '/count', String, PrintRow2)
	#rospy.Subscriber('/abacus/row' + str(2) + '/count', String, PrintRow3)

	if groupNumber == "AE":
		for j in range(1,4):
			prodSeries = j
			ProductionTask(prodSeries)
			print ("finishProTask: " + str(finishProTask))
			if finishProTask:
				#if j == 1:
				#	ComprehensionTask("ambiguous")
				#elif j == 2:
				#	ComprehensionTask("egocentric")
				#	if finishCompTask:
				#		orderCounter = j+1
				#else:
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

#def print_location(data):
	#print ("-------------------------------------------------")
	#print (data)

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

def PrintBasicData(data, rowN):
	global yekan
	global dahgan
	global sadgan
	#print ("-------------------------------------------------")
	if rowN == 0:
		yekan = int(data.data)
	elif rowN == 1:
		dahgan = int(data.data)
	elif rowN == 2:
		sadgan = int(data.data)
	#print data
	return (int (data.data))

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
		PointAtTheTablet("Right", h, turn + 1, 2, stay)
		
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



def PointAtTheTablet(direction, step, turn, row, stay):
	"""
	The function that carry out true pointing
	"""
	global tracker
	global frame

	frame      = motion.FRAME_ROBOT
	
	effector = 'RArm'
	maxSpeedHead = 0.2
	useWholeBody = False
	useSensorValues = True

	XA = 0.2
	XB = -0.2
	Slice = float((XA - XB)/turn)
	print ("slice-----------", Slice)
	print ("turn-----------", turn)

	if(effector == "RArm"):
		motionProxy.openHand("RHand")
	else:
		motionProxy.openHand("LHand")

	if direction == "Left":
	 	position1 = [0.3 + row * 0.2, XA - Slice * step, 0.1]
	 	position2 = [0.7, 0, 0.1]
		position3 = [0.7, -0.2, 0.1]
	elif direction == "Right":
		position1 = [0.3 + row * 0.2, XB + Slice * step, 0.1]
		position2 = [0.7, 0, 0.1]
		position3 = [0.7, 0.2, 0.1]
		
	curPose = motionProxy.getPosition("RHand", frame, useSensorValues)
	print ("position-------------------------", position1)
	print ("direction-------------------------", direction)

	tracker.post.lookAt(position1, frame, maxSpeedHead, useWholeBody)
	tracker.pointAt("RArm", position1, frame, maxSpeedHead)
	"""print "pointing "
	tracker.post.lookAt(position2, frame, maxSpeedHead, useWholeBody)
	tracker.pointAt(effector, position2, frame, maxSpeedHead)

	tracker.post.lookAt(position3, frame, maxSpeedHead, useWholeBody)
	tracker.pointAt("LArm", position3, frame, maxSpeedHead)"""
	# 
	#curPosition = [curPose[0], curPose[1], curPose[2]] 
	#tracker.pointAt(effector, curPose, frame, maxSpeedHead)
	if step == turn and not stay:
		postureProxy.goToPosture("Crouch", 0.3)



def ProductionInteractionScenario(prodSeries, proOrder, productionBatch):
	# Robot looks at the child when the child talks
	# robot looks at the abacus
	# and when the child asks what is the number? robot answers based on the reading from the file
	nbBead = 10
	turn = 5
	stay = False


	# CALCULATION
	#######################################################################
	reset_publish = rospy.Publisher('reset/order', String, queue_size=1)
	reset_publish.publish("1")

	# The numbers that the child has in their card
	#------------------- 
	currenetElements = [int(i) for i in productionBatch[proOrder-1]]
	print "currenetElements"
	print currenetElements



	# Subscribe to the max diatance and the size of the beads recieved from the virtual abacus side
	
	rospy.Subscriber('/abacus/bead/radius', String, PrintRadius)
	rospy.Subscriber('/abacus/row/length', String, PrintLength)
	
	beadRadius = basic_data_dict['radius']
	maxRowLength = basic_data_dict['length']
	#maxRowLength = 1500
	#beadRadius = 100
	print ("maxRowLength", maxRowLength)
	print ("beadRadius", beadRadius)



	# ACTION (old Scenario)
	#######################################################################
	# The child asks the robot which number the abacus shows?
	# the robot says the right number according to 

	# ACTION 
	#######################################################################
	# Robot looks at the child
	#-------------------------------------
	LookAtTheChild(True)
	
	# Child tells the robot to show a number
	# child uses a card to give instruction
	# robot knows that number based on the production_series data set
	# the numbers for each trial is stored in currentElements 
	#------------------------------------
	time.sleep(2)
	firstValue = currenetElements[0]

	# robot acts like it has heard the child, so now it looks at the tablet 
	# and shows the number requested
	# it also moves its hand to point at it 
	LookAtTheChild(False)
	reset_publish.publish("0")
	print ("reset")
	time.sleep(2)

	print("robot looking at the tablet")
	LookAtTheTablet()
	#PointAtTheTablet("Right")
	lookingDirection = MoveBeadPublisher(firstValue, maxRowLength, beadRadius, turn, stay)

	#rospy.Subscriber("target_pose", PoseArray, MoveRobotArm)

	# Function for the robot to move its hand

	# Function for the robot to move its head

	# robot looks at the child again
	#------------------------------------------
	LookAtTheChild(True)
	time.sleep(5)
	secondValue = currenetElements[1]


	# CALCULATION
	##########################################################################
	# Now it is the time to solve the addition or subtraction question
	# in this stage there are 5 trials, 
	# robot answers to the first and fifth ones correctly
	# and it randomly answers to two questions wrongly among the 2nd to 4th one

	# Making decision on how to make a wrong answer
	# ---------------------------------------------------- 
	if firstValue > secondValue:
		wrongSecondValue = -1 * secondValue
	else:
		wrongSecondValue = random.choice([x for x in range(secondValue-2, secondValue+3) if x != secondValue])

	print "firstValue = " + str(firstValue)
	print "secondValue = " + str(secondValue)
	print "wrongSecondValue = " + str(wrongSecondValue)


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
	# robot moves the second value according to the decison based on the order of the trial
	LookAtTheChild(False)
	time.sleep(2)

	LookAtTheTablet()
	# When the answer is less than 10
	if XsecondValue + firstValue < nbBead:
		stay = False
		#if XsecondValue<0:
		#	PointAtTheTablet("Left")
		#else:
		#	PointAtTheTablet("Right")
		MoveBeadPublisher(XsecondValue, maxRowLength, beadRadius, turn, stay, firstValue)
		#rospy.Subscriber("target_pose", PoseArray, MoveRobotArm)


	# When the answer is more than 10
	if XsecondValue + firstValue >= nbBead:
		belowTen = nbBead - firstValue
		stay = True
		# Move The beads in the first row to get 10
		#PointAtTheTablet("Right")
		MoveBeadPublisher(belowTen, maxRowLength, beadRadius, turn, stay, firstValue )
		
		time.sleep(0.5)
		# Move the beads back to initial position in the first row
		#MoveBeadBackPublisher(belowTen, maxRowLength, beadRadius, firstValue)
		#reset_publish_row0 = rospy.Publisher('reset/row0/order', String, queue_size=1)
		#PointAtTheTablet("Left")
		
		PointAtTheTablet("Left", 1, 2, 1, stay)
		reset_publish.publish("1")
		PointAtTheTablet("Left", 2, 2, 1, stay)

		time.sleep(0.5)
		# Move one bead from the second row to count the 10
		#PointAtTheTablet("Right")
		MoveBeadPublisherRow2(1, maxRowLength, beadRadius, turn, stay, 0, 1)
		AboveTen = XsecondValue - belowTen
		time.sleep(1.5)
		# Move the rest of the beads in the first row to find the final answer
		stay = False
		MoveBeadPublisher(AboveTen, maxRowLength, beadRadius, turn, stay)


	time.sleep(5)



	# At this point the robot tells the final answer to the child
	LookAtTheChild(True)
	communicate.say ("The answer is " + str(answerValue))
	communicate.say ("Can you tell me if my answer was correct?")
	time.sleep(2)



	# Subscribe to the event of child's evaluation of the robot's answer
	# Child's response will come from the abacus side
	#----------------------------------------------------------------
	#child_response = rospy.Subscriber('button/response', String, PrintBasicData)
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
		communicate.say ("I am so so happy?")
	elif not child_response:
		communicate.say ("I am sorry, I will try to be more careful")


	if proOrder == 5:
		communicate.say ("I sloved a lot of questions, now it is your turn to solve my questions")
	else:
		communicate.say ("Let's solve another question")





	"""
	valueOnAbacus = random.sample(range(1,9), 1)
	valuePlanned = currenetElements [0]

	valueOnAbacus = valuePlanned

	print "valueOnAbacus = " + str(valueOnAbacus)
	print "valuePlanned = " + str(valuePlanned)

	if valueOnAbacus == valuePlanned:
		#communicate.say ("I can see the abacus shows " + str(valueOnAbacus))
		print ("I can see the abacus shows " + str(valuePlanned))
		firstValue = valuePlanned

	else:
		print ("The abacus shows " + str(valueOnAbacus[0]) + ", is that correct?")
		firstValue = valueOnAbacus[0]
		#communicate.say ("The abacus shows" + valueOnAbacus )

	# Now the child instruct the robot to add or deduct a number from the current value.
	# Robot listening"""
	
	

def ComInteractionScenario():
	h = 0 





def ComprehensionTask(compTypeSelection):
	global finishCompTask

	comprehension_numbers = rospy.get_param('~comprehension_numbers_series1')
	numbersArray = comprehension_numbers.splitlines()
	print "numbersArray"
	print numbersArray

	elements = []
	for line in numbersArray:
		elements.append(re.split('\W+,', line))
	elements = elements[2:]

	print ("elements: ")
	print elements

	compType = compType_dict[compTypeSelection]
	#print 

	compOrder = 1

	if compType == 1:
		comprehensionBatch = elements[(compType - 1)*5: (compType)*5]
		print ("comprehensionBatch")
		print (comprehensionBatch)
		time.sleep(1)

		for k in range (5):
			time.sleep(1)
			AmbigiousBehaviour(comprehensionBatch, k)

	elif compType == 2:
		comprehensionBatch = elements[(compType - 1)*5 + 1: (compType)*5 + 1]
		print ("comprehensionBatch")
		print (comprehensionBatch)
		time.sleep(1)

		for k in range (5):
			time.sleep(1)
			EgocentricBehaviour(comprehensionBatch, k)

	elif compType == 3:
		time.sleep(1)
		comprehensionBatch = elements[(compType - 1)*5 + 2: (compType)*5 + 2]
		print ("comprehensionBatch")
		print (comprehensionBatch)

		for k in range (5):
			time.sleep(1)
			AddresseeBehaviour(comprehensionBatch, k)

	finishCompTask = True




def AmbigiousBehaviour(comprehensionBatch, compOrder):
	global confusion_mat_comp_pub	
	global yekan
	global dahgan
	global sadgan
	#communicate.post.say("\\rspd=90\\ can you move two beads to the right?")

	currenetElements = [int(i) for i in comprehensionBatch[compOrder]]
	print ("currenetElements")
	print (currenetElements)
	firstValue = currenetElements[0]
	secondValue = currenetElements[1]
	answerValue = currenetElements[2]

	print (" First, show me " + str(firstValue) + " on the abacus")
	communicate.say (" First, show me " + str(firstValue) + " on the abacus")
	
	# Check on the child answer that comes from the abacus
	# Recieves the current count from the abacus side of the code 
	#- --------------------------------------------------------------
	#rospy.Subscriber('/abacus/row' + str(0) + '/count', String, PrintRow1, 0)
	#rospy.Subscriber('/abacus/row' + str(1) + '/count', String, PrintRow2, 1)
	#rospy.Subscriber('/abacus/row' + str(2) + '/count', String, PrintRow3, 2)
	yekan = row_count_dict['yekan']
	dahgan = row_count_dict['dahgan']

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



"""def IntroduceNao(numOfStart, childName):
	
	#Nao starts introducing itself when the book cover is in front of him 
	
	global robot_state_pub
	global blinkThread 
	facesize = 0.1

	robot_state_pub.publish("Introduce Nao Started")
	# First, wake up
	#postureProxy.goToPosture("Crouch", 0.5)

	if numOfStart == "1":
		LookAtTheTablet(0.3, -0.8)
		faceTrackingStarted(facesize)
	turn_on_eye()
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	blinkingModeON("ON")
	mode = "ON"

	
	communicate.setLanguage('English')
	if numOfStart == "1":
		script =   ("\\rspd=90\\ Hello " + childName + " \\pau=500\\ "
					"\\rspd=90\\ let's start wokring woth abacus? \\pau=500\\"
					"\\rspd=90\\ I'm so excited")
	else:
		script = ("\\rspd=90\\ ")
	communicate.say(script)
	#communicate.say("\\rspd=90\\ Hello everyone \\pau=700\\ My name is Nao \\pau=500\\ I really like reading short stories")
	#communicate.say("\\rspd=90\\ Do you want to listen to them?")
	#communicate.say("\\rspd=90\\ sometimes I make mistakes, can you help me to correct them?")
	#time.sleep(0.1)
	#communicate.say("\\rspd=90\\ If you want to read with me, please bring the book")
	#communicate.say("\\rspd=90\\ and don't forget the red, green and yellow card")
	#communicate.say("\\rspd=90\\ \\pau=50\\ you can show me the red card if I make a mistake")
	#communicate.say("\\rspd=90\\ \\pau=50\\ the green card when I'm correct")
	#communicate.say("\\rspd=90\\ \\pau=50\\ and the yellow one, when you want me to repeat")
	#communicate.say("\\rspd=90\\ Hello")
	robot_instruction_pub.publish(script)
	pitch_angle = 0.2
	time.sleep(0.5)
	#idleMovementModeON("ON")
	robot_state_pub.publish("Introduce Nao Ended")"""

def IntroduceNao(numOfStart,childName):
	if numOfStart == "1":
		print ("Hello " + childName + "\\pau=700\\ Welcome back \\pau=500\\")
		#communicate.say("\\rspd=90\\ Hello everyone \\pau=700\\ My name is Nao")
		postureProxy.goToPosture("Crouch", 0.5)






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
	initialization_dict["MoveHandSpeed"] = instructionArray[5]
	initialization_dict["ReadDelay"] = instructionArray[7]
	initialization_dict["ReadSpeed"] = instructionArray[9]
	initialization_dict["ReactionDelay"] = instructionArray[11]
	initialization_dict["HandPoint"] = instructionArray[13]
	initialization_dict["WhichPage"] = instructionArray[15]
	initialization_dict["CorrectionState"] = instructionArray[17]
	initialization_dict["BookSelection"] = instructionArray[19]





	

	#maxSpeedHead = 0.1
	# Initialize the log file
	logFile = logger()
	time.sleep(5)
	robot_instruction_pub.publish(instructionText)
	time.sleep(1)

	childName = initialization_dict["ChildName"]

	child_name_pub.publish("Sierra")
	print "childName"
	correction_state_pub.publish("correct")

	#robot_speed_pub.publish(str(initialization_dict["MoveHandSpeed"]) + "with" + str(initialization_dict["MoveHandSpeed"]) + "delay")
	# Face tracking activated
	#faceTrackingEnded()
	#enabled_state_pub.publish("Face Tracking is OFF")
	facesize = 0.1
	global restingEnabled 
	restingEnabled = False
	#faceTrackingStarted(facesize)
	#enabled_state_pub.publish("Face Tracking is ON")
	
	global space
	space      = motion.FRAME_ROBOT


	global RobotConnected
	RobotConnected = True



	# The function to introduce Nao and stop the face tracking 
	time.sleep(0.5)
	IntroduceNao(numOfStart, childName)
	#faceTrackingEnded()
	#enabled_state_pub.publish("Face Tracking is OFF")

	# Make the robot to look at the text
	pitch_angle = 0.3
	yaw_angle = 0.8
	#LookAtTheBook(pitch_angle)
	

	# Subscribe to the tags
	#rospy.Subscriber('tag_id_state', String, tagDetection)

	# Something that can be stated in the initializationFile
	groupNumber = "AE"
	orderCounter = 0
	productionSeries = 0
	##### Start the assignment of tasks
	TaskAssignment(groupNumber, orderCounter, productionSeries)



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

