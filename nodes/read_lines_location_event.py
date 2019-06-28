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


story_dict ={	'[200, 201]': 1, 
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
effector = "LArm"
global once
once = 0
#global mistakeExist
#mistakeExist = True

class SpeechModule(ALModule):
    """ A simple module able to react
    to facedetection events

    """

    def onTextDone(self, strVarName, value, strMessage):
		""" This will be called each time a face is
		detected.

		"""
		# Unsubscribe to the event when talking,
		# to avoid repetitions
		global SpeechDone
		if value == 0:
			SpeechDone = False
		elif value == 1:
			SpeechDone = True
		#print "SpeechDone"
		#print SpeechDone

        # Subscribe again to the event
        #memory.subscribeToEvent("ALTextToSpeech/TextDone", "SpeechCondition", "onTextDone")

def fonTextDone(value):
    # Called when NAO begins or ends the speech. On end the value = 1
    global SpeechDone
    # Must work only on speech with feedback mode
    if value == 0:
    	SpeechDone = False
    elif value == 1:
    	SpeechDone = True
    #print "SpeechDone"
    #print SpeechDone


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




def cardPointedAt(msg):
	"""

	"""
	global cardPosition
	cardPosition += 1


	if cardPosition == 10:

		cameraName = 'CameraBottom'
		#changeCoordinate = TRANSFORMATION(motionProxy)
		#pointedWord = Pose()
		#pointedWord.position = msg.position
		PW = []
		PW.append([msg.position.x])
		PW.append([msg.position.y])
		PW.append([msg.position.z])
		PW.append([1])

		#P1 = changeCoordinate.transformPoint(PW, cameraName)




global breakPoint
breakPoint = [0, 0]

def changeToPose(res):
	middle_pose = Pose()
	middle_pose.position.x = res[0]
	middle_pose.position.y = res[1]
	middle_pose.position.z = res[2]
	middle_pose.orientation.x = res[3]
	middle_pose.orientation.y = res[4]
	middle_pose.orientation.z = res[5]
	middle_pose.orientation.w = 1

	return middle_pose

global speech_with_feedback_flag



def calculateWhereToPointAt(pointsAB, effector, frame, changeCoordinate):

	global correctPermission
	global trajComplete
	global story
	trajComplete = []
	global ARTag
	global REDButtonAllert
	global YELLOWButtonAllert
	global reactionPermission
	global breakPoint
	global robot_hand_pose_pub
	global lastBreakPoint
	global maxSpeedHead
	global SpeechDone
	SpeechDone = False
	#global mistakeExist
	useSensorValues = True
	handPoint = bool(initialization_dict["HandPoint"])
	readSpeed = initialization_dict["ReadSpeed"]

	bookNumber = int(initialization_dict["BookSelection"])

	if handPoint == True:
		robot_gesture_pub.publish("Pointing")
	elif handPoint == False:
		robot_gesture_pub.publish("Not Pointing")

	if(effector == "LArm"):
		whichHand = "LHand"
	else:
		whichHand = "RHand"

	for j in range(len(ARTag)):

		wordProc = WORDPROCESSING(story, ARTag[j], taskLevel_dict, proxy, readSpeed, bookNumber)

		#INTag = "=WordNum"
		#wordCount = wordProc.getTheInstructionTagData(INTag)
		INTag = "=LineNum"
		lineCount = wordProc.getTheInstructionTagData(INTag)
		INTag = "=Skip"
		skipOrNot = wordProc.getTheInstructionTagData(INTag)
		INTag = "=Read"
		readOrNot = wordProc.getTheInstructionTagData(INTag)
		LiWoCount2 = []
		lineDistanceCoef = []
		#if lineCount > 1:
		for i in range(lineCount):
			INTag = "=L" + str((i+1))
			LiWoCount2.append(wordProc.getTheInstructionTagData(INTag))

			INTag = "=S" + str((i+1))
			lineDistanceCoef.append(0.01 * wordProc.getTheInstructionTagData(INTag))

		LiWoCount = wordProc.getNumberOfWords(lineCount)


		wordProc.toCorrectOrNotTo(ARTag)



		wordProc.getTheLineMatrix()
		#wordCount = wordProc.getTheWordCount()
		wordProc.clearAllTheInstructionTags()

		#changeCoordinate = TRANSFORMATION( motionProxy)
		#wordProc = WORDPROCESSING()

		maxSpeed = 0.1
		maxSpeedHead = float(initialization_dict["MoveHandSpeed"])
		#maxSpeedHead = 0.1
		useWholeBody = False

		trajComplete2 = []

		PA = Pose()
		PB = Pose()
		PC = Pose()
		PD = Pose()
		
		PA.position.x = pointsAB[2*j].item(0, 0)
		PA.position.y = pointsAB[2*j].item(1, 0)
		PA.position.z = pointsAB[2*j].item(2, 0)

		PB.position.x = pointsAB[2*j+1].item(0, 0)
		PB.position.y = pointsAB[2*j+1].item(1, 0)
		PB.position.z = pointsAB[2*j+1].item(2, 0)
		trajComplete2 = changeCoordinate.calculateEachWordPosition(PA, PB, effector, LiWoCount, lineDistanceCoef)


		hardWord = 4 

		global current_word_pub
		global robot_state_pub
		robot_state_pub.publish("Reading Mode Started")

		if correctPermission == True:
			correction_state_pub.publish("True")
		elif correctPermission == False:
			correction_state_pub.publish("False")
		else:
			correction_state_pub.publish(correctPermission)

		

		#wordProc.readTheTaggedStory(correctPermission)
		curPose          = motionProxy.getPosition(whichHand, frame, useSensorValues)
		"""if taskLevel_dict['CurrLevel'] == "TaskONE":
			for i in range(len(trajComplete2)):
				for j in range(len(trajComplete2[i][0])):
					if handPoint == True:
						if i == 0 and j == 0:
							tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)
					if (j ) %4 == 0:
						correctFlag = wordProc.readFromWordMatrix(correctPermission, j/4, i+1, breakPoint, REDButtonAllert, YELLOWButtonAllert, taskLevel_dict['CurrLevel'], ARTag, readSpeed, current_word_pub)
					"""

		if taskLevel_dict['CurrLevel'] == "TaskONE":
			if readOrNot == 1:
				for i in range(len(trajComplete2)):

					for j in range(len(trajComplete2[i][0])):
						#if stop_state["StopReading"] == True:
							#break
						if REDButtonAllert == True:
							break
						if YELLOWButtonAllert == True:
							break
				
						#if handPoint == True:
							#if breakPoint != [0, 0]:
							#	if [j, j] >= breakPoint:
							#		tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
							#		tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)
							#else:
						tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
						if handPoint == True:
							#if not (i == 0 and j == 0):
							tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)
							#current_position          = motionProxy.getPosition(whichHand, frame, useSensorValues)
						#elif handPoint == False:
							#time.sleep(float(initialization_dict["ReadDelay"]) + 0.01)
							#maxSpeedHead = 0.02
							#tracker.pointAt(effector, [curPose[0], curPose[1], curPose[2]], frame, maxSpeedHead/2)

						#elif 
							#tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
							#postureProxy.goToPosture("Crouch", 1)

						if j == 0:
							#wordProc.readFromWordMatrix(correctPermission, j, i+1, REDButtonAllert)
							if REDButtonAllert == True:
								break
							if YELLOWButtonAllert == True:
								break						
							#wordProc.readFromMatrixLine(correctPermission, i+1, REDButtonAllert)
							x = True
							"""if REDButtonAllert == True:
								break
							if YELLOWButtonAllert == True:
								break """
						if (j ) %4 == 0:

							result          = motionProxy.getPosition(whichHand, frame, useSensorValues)
							handPose = changeToPose(result)
							robot_hand_pose_pub.publish(handPose)		

							correctFlag = wordProc.readFromWordMatrix(correctPermission, j/4, i+1, breakPoint, REDButtonAllert, YELLOWButtonAllert, taskLevel_dict['CurrLevel'], ARTag, readSpeed, current_word_pub)
							currReadingPoint = [i+1, j/4]

							#Sleepy
							pause = float(initialization_dict["ReadDelay"])
							subscriber = memory.subscribeToEvent("ALTextToSpeech/TextDone", "SpeechCondition", "onTextDone")
							while not SpeechDone:
								#print "What to do"
								#print SpeechDone
								time.sleep(0.001)
							if SpeechDone:
								SpeechDone = False
								continue 

							time.sleep(pause)
							robot_state_pub.publish(str(pause))

							if REDButtonAllert == True:
								break
							if YELLOWButtonAllert == True:
								break 
						#if j  == 4:
							#wordProc.readAllTogether(REDButtonAllert, YELLOWButtonAllert)
							#print "finish"



			elif readOrNot == 0:
				animationSelection = MOTION_ANIMATION_SELECTION()
				animationSelection.reactionToSpecificTags()
				correctFlag = True

			x = len(trajComplete2)-1
			if currReadingPoint == [len(trajComplete2) , len(trajComplete2[x][0])/4]:
				stop_state["FinishReading"] = True

			if REDButtonAllert == True:
				correctPermission = "Partial Correction"
				breakPoint = currReadingPoint
				lastBreakPoint = currReadingPoint
				break

			if YELLOWButtonAllert == True:
				if correctPermission == "Partial Correction":
					breakPoint = lastBreakPoint
				elif (correctPermission == True or correctPermission == False):
					breakPoint = [0, 0]
					lastBreakPoint = [0, 0]
				break

				
			if correctFlag == "Partial Correction":
				correctPermission = "Partial Correction"
				breakPoint = [0, 0]


			elif correctFlag == True:
				correctPermission = True
				breakPoint = [0, 0]
				lastBreakPoint = [0, 0]

		print "here"
		time.sleep(0.1)		
		# Empty the arrays
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)
		print "skipOrNot"
		print skipOrNot
		if skipOrNot == 0:
			time.sleep(0.5)
			if correctPermission == False:
				#valuationAfterFirst() 
				print "no evaluation"

			elif correctPermission == True:
				valuationAfterRepeat()
		elif skipOrNot == 1:
			print "skip"

		trajComplete2 = []
		wordProc.cleanStory()
		changeCoordinate.clearCoordinate()
		reactionPermission = True
		blinkingModeON("ON")
		idleMovementModeON("ON")

def valuationAfterFirst():
	Num = random.randint(1,5)

	if Num == 1:
		story.post.say("\\rspd=90\\ did I read it \\pau=40\\ correctly?") 

	if Num == 2:
		story.post.say("\\rspd=80\\ what do you think? \\pau=40\\ was I good?")

	if Num == 3:
		story.post.say("\\rspd=80\\ Should I read it again?")

	if Num == 4:
		story.post.say("\\rspd=80\\ can I know your opinion?")

	if Num == 5:
		story.post.say("\\rspd=80\\ I'm done, was it good?")

	if Num == 6:
		story.post.say("\\rspd=80\\ did I read it \\pau=50\\ correctly?")



def valuationAfterRepeat():
	Num = random.randint(1,5)

	if Num == 1:
		story.post.say("\\rspd=80\\ what about \\pau=20\\ this time?")  

	if Num == 2:
		story.post.say("\\rspd=80\\ I hope I read it correctly?")

	if Num == 3:
		story.post.say("\\rspd=80\\ did I correct my mistake?")

	if Num == 4:
		story.post.say("\\rspd=80\\ Did I make a mistake again?")

	if Num == 5:
		story.post.say("\\rspd=80\\ can I know your opinion again?")



def stopAndAsk():

	story.say("\\rspd=80\\ Oh")
	story.say("\\rspd=80\\ I don't know how to read this word") 
	story.say("\\rspd=80\\ can you help me?")
	# activate Speech recognition
	# record the audio
	# get the text back 
	if empty:
		story.say("\\rspd=80\\ sorry I couldn't hear you?")

	if full:
		if confidenceLevel >= threshold:
			story.say("\\rspd=80\\ Oh! now I understood. It is " + recognizedWord + ".")
		elif confidenceLevel < threshold:
			story.say("\\rspd=80\\ did you say" + recognizedWord + "?")





def getTagLocations(msg):
	
	global trajComplete
	#trajComplete = [1]
	global ARTag
	changeCoordinate = TRANSFORMATION(motionProxy)
	global moveHand
	moveHand += 1
	cameraName = 'CameraBottom'
	global whichPage
	#whichPage = "Left"



	if moveHand == 1:
		#whichPage == "Left"
		k = 0
		if whichPage == "Left":
			k = 0
			effector = "LArm"

		elif whichPage == "Right":
			k = 2
			effector = "RArm"


		if(effector == "LArm"):
			motionProxy.openHand("LHand")
		else:
			motionProxy.openHand("RHand")


		
		P1 = []
		P1.append([msg.poses[k].position.x])
		P1.append([msg.poses[k].position.y])
		P1.append([msg.poses[k].position.z])
		P1.append([1])
		P2 = []
		P2.append([msg.poses[k+1].position.x])
		P2.append([msg.poses[k+1].position.y])
		P2.append([msg.poses[k+1].position.z])
		P2.append([1])


		""" Transformation Matrix"""
		#tranAC = changeCoordinate.transformMatrix(cameraName)
		newP1 = changeCoordinate.transformPoint(P1, cameraName)
		newP2 = changeCoordinate.transformPoint(P2, cameraName)

		""" to make sure the robot's hand move from left to write
			start from the tag located in the left toward the one in the right
			compare their y value
		"""
		pointsAB = []
		if newP1[1] >= newP2[1]:
			pointsAB.append(newP1)
			pointsAB.append(newP2)
		else:
			pointsAB.append(newP2)
			pointsAB.append(newP1)
		


		calculateWhereToPointAt(pointsAB, effector, space, changeCoordinate)

		#time.sleep(2)
		postureProxy.goToPosture("Crouch", 0.5)
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)
		pairs_dict[tag_pairs] = True



def readAndMoveInstruction( cameraName, handMovePermission, detected_tag):

	global reactionPermission
	global ARTag
	ARTag = []
	ARRec = detected_tag
	ARRec2 = ARRec.replace('[', '').replace(']', '')
	foundAR = re.split(",", ARRec2)
	if len(foundAR) == 2:
		tagNum = 2
		ARTag.append(foundAR[0] +","+foundAR[1])

	elif len(foundAR) == 4:
		tagNum = 4
		ARTag.append(foundAR[0] +","+foundAR[1])
		ARTag.append(foundAR[2] +","+foundAR[3])
	#wordProc = WORDPROCESSING(story, ARTag)
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', False)
	#motionProxy.setBreathEnabled('Head', False)
	if handMovePermission == True:
		reactionPermission = False
		blinkingModeOFF()
		idleMovementModeOFF()


		rospy.Subscriber("target_pose", PoseArray, getTagLocations)

		#motionProxy.setBreathEnabled('Arms', True)

global delaytime
delaytime = 0


def tagDetection(msg):
	"""

	"""
	#faceTrackingEnded()
	global delaytime
	#global wordCount
	global moveHand
	global correctPermission
	global reactionPermission
	global REDButtonAllert
	global YELLOWButtonAllert
	global whichPage
	global breakPoint
	global NumOfGreenCard
	global ARRec
	global robot_state_pub
	global once
	global FinishSign
	global detected_ARtags_pub
	global mistakeExist
	childName = initialization_dict["ChildName"]
	#global effector
	animationSelection = MOTION_ANIMATION_SELECTION()
	
	# initializing classes

	taskSelection()
	if taskLevel_dict['CurrLevel'] == "TaskFINISH":
		once += 1
		GoToFinishLine(once, childName)
		
	cameraName = 'CameraBottom'

	#if taskLevel_dict['CurrLevel'] == "TaskFINISH":
		#break

	# Tilt the roobot's head to the front
	pitch_angle = 0.3
	global counter
	if counter == 0:
		yaw_angle = 0.3
		LookAtTheBook(pitch_angle)
		counter = 1


	ARRec = msg.data
	#robot_state_pub.publish("Tags " + ARRec + " Detected")
	ARRec2 = ARRec.replace('[', '').replace(']', '')
	foundAR = re.split(",", ARRec2)

	detected_ARtags_pub.publish(ARRec2)


	if len(foundAR) == 2:
		tagNum = 2
		RecTag = msg.data
		tagLeft = msg.data
		whichPage = "Left"


	elif len(foundAR) == 4:
		tagNum = 4
		tagLeft	 = '[' + foundAR[0] + "," + foundAR[1] + ']'
		tagRight = '[' + foundAR[2] + "," + foundAR[3] + ']'




		if (pairs_dict[tagLeft] == True or pairs_dict[ARRec] == "Switch" or pairs_dict[ARRec] == "Right" or initialization_dict["WhichPage"] == "Right"):
			RecTag = tagRight
			whichPage = "Right"
			pairs_dict[ARRec] = "Right"
			initialization_dict["WhichPage"] = "Left"
			
			#effector == "RArm"
		else:
			RecTag = tagLeft
			whichPage = "Left"
			pairs_dict[ARRec] = "Left"
			delaytime += 1
			#effector == "LArm"


	if initialization_dict["CorrectionState"] == "True":
		pairs_dict[RecTag] = "Correcting Mode"
		correctPermission = True
		initialization_dict["CorrectionState"] = "Default"

	if (RecTag == "[110, 111]" or RecTag == "[ 55, 56]" or RecTag == "[134, 135]" or RecTag == "[ 100, 101]"):
		taskLevel_dict['CurrLevel'] = "TaskFINISH"

	if taskLevel_dict['CurrLevel'] == "TaskFINISH":
		once += 1
		GoToFinishLine(once, childName)


	for i in pairs_dict:
		if not (i == RecTag or i == tagLeft):
			if pairs_dict[i] == "Waiting for Red-card":
				pairs_dict[i] = False

			elif pairs_dict[i] == True:
				pairs_dict[i] = "Corrected Mode"

			elif pairs_dict[i] == "Waiting for Green-card":
				pairs_dict[i] = "Corrected Mode"

			#elif pairs_dict[i] == "Correct Happy Mode":
				#pairs_dict[i] = "Corrected Mode"

			#elif pairs_dict[i] == "Wrong Happy Mode":
				#pairs_dict[i] = False
			
			elif pairs_dict[i] == "Repeat Correct Mode":
				pairs_dict[i] = "Corrected Mode"

			elif pairs_dict[i] == "Repeat Wrong Mode":
				pairs_dict[i] = False

			elif pairs_dict[i] == "Repeated Correct Mode":
				pairs_dict[i] = "Corrected Mode"	

			elif pairs_dict[i] == "Repeated Wrong Mode":
				pairs_dict[i] = False					



	#rospy.Subscriber('card_id_state', String, cardDetection, RecTag)
	rospy.Subscriber('pushedR', Bool, ButtonDetectionR, RecTag)
	rospy.Subscriber('pushedG', Bool, ButtonDetectionG, RecTag)
	rospy.Subscriber('pushedY', Bool, ButtonDetectionY, RecTag)

	detected_ARtags_pub.publish(RecTag) 

	if pairs_dict[RecTag] == True:
		page_state_pub.publish("True")
	elif pairs_dict[RecTag] == False:
		page_state_pub.publish("False")
	else:
		page_state_pub.publish(pairs_dict[RecTag])

	if pairs_dict[RecTag] == "FinishSign":
		once += 1
		GoToFinishLine(once)


	if pairs_dict[RecTag] == False:
		moveHand = 0
		correctPermission = False
		handMovePermission = True
		stop_state["FinishReading"] = False
		readAndMoveInstruction(cameraName, handMovePermission, RecTag)

		reactionPermission = True
		REDButtonAllert = False
		YELLOWButtonAllert = False
		pairs_dict[RecTag] = "Waiting for Red-card"


		"""if mistakeExist:
			pairs_dict[RecTag] = "Waiting for Red-card"

		elif not mistakeExist:

			pairs_dict[RecTag] = "Waiting for Green-card"
		"""

	elif pairs_dict[RecTag] == "Correcting Mode":
				
		if reactionPermission == True:
			animationSelection.reactionToREDCard()
			moveHand = 0
			#correctPermission = True
			handMovePermission = True
			REDButtonAllert = False
			YELLOWButtonAllert = False
			stop_state["FinishReading"] = False
			readAndMoveInstruction(cameraName, handMovePermission, RecTag)
			pairs_dict[RecTag] = "Waiting for Green-card"
			if correctPermission == "Partial Correction":
				pairs_dict[RecTag] = "Waiting for Red-card"
				#breakPoint = [0, 0]
			elif correctPermission == True:
				breakPoint = [0, 0]
				pairs_dict[RecTag] = "Waiting for Green-card"

		
		elif reactionPermission == False:
			#animationSelection.reactionToREDCard()
			moveHand = 0
			#correctPermission = True
			handMovePermission = True
			REDButtonAllert = False
			YELLOWButtonAllert = False
			stop_state["FinishReading"] = False
			readAndMoveInstruction(cameraName, handMovePermission, RecTag)
			pairs_dict[RecTag] = "Waiting for Red-card"
		#pairs_dict[msg.data] = True
		#pairs_dict[RecTag] = "Waiting for Green-card"


	elif pairs_dict[RecTag] == "Correct Happy Mode":
		pairs_dict[RecTag] = True
		motionProxy.setBreathEnabled('Arms', False)
		if reactionPermission == True:
			NumOfGreenCard += 1
			animationSelection.reactionToGREENCard()
		#story.say("\\rspd=70\\ Lets go to the next page \\pau=500\\ ")


	elif pairs_dict[RecTag] == "Wrong Happy Mode":
		pairs_dict[RecTag] = True
		motionProxy.setBreathEnabled('Arms', False)
		if reactionPermission == True:
			animationSelection.reactionToGREENCard()
		#story.say("\\rspd=70\\ Lets go to the next page \\pau=500\\ ")

	

	elif pairs_dict[RecTag] == "Corrected Mode":
		moveHand = 0
		correctPermission = True
		handMovePermission = True
		REDButtonAllert = False
		YELLOWButtonAllert = False
		stop_state["FinishReading"] = False
		readAndMoveInstruction(cameraName, handMovePermission, RecTag)
		#pairs_dict[msg.data] = True
		pairs_dict[RecTag] = "Waiting for Green-card"



	elif pairs_dict[RecTag] == "Repeat Correct Mode":	
		moveHand = 0
		correctPermission = True
		handMovePermission = True
		REDButtonAllert = False
		YELLOWButtonAllert = False
		stop_state["FinishReading"] = False
		readAndMoveInstruction(cameraName, handMovePermission, RecTag)
		if correctPermission == "Partial Correction":
			pairs_dict[RecTag] = "Waiting for Green-card"
			#breakPoint = [0, 0]
		elif correctPermission == True:
			breakPoint = [0, 0]
			pairs_dict[RecTag] = "Waiting for Green-card"
		elif correctPermission == False:
			breakPoint = [0, 0]
			pairs_dict[RecTag] = "Waiting for Red-card"
		#pairs_dict[RecTag] = "Repeated Correct Mode"



	elif pairs_dict[RecTag] == "Repeat Wrong Mode":	
		moveHand = 0
		#correctPermission = False
		handMovePermission = True
		REDButtonAllert = False
		YELLOWButtonAllert = False
		stop_state["FinishReading"] = False
		readAndMoveInstruction( cameraName, handMovePermission, RecTag)
		if correctPermission == "Partial Correction":
			pairs_dict[RecTag] = "Waiting for Red-card"
			#breakPoint = [0, 0]
		elif correctPermission == True:
			breakPoint = [0, 0]
			pairs_dict[RecTag] = "Waiting for Green-card"
		elif correctPermission == False:
			breakPoint = [0, 0]
			pairs_dict[RecTag] = "Waiting for Red-card"



	elif pairs_dict[RecTag] == "Repeated Wrong Mode" :
		moveHand = 1
		pairs_dict[RecTag] = "Waiting for Red-card"



	elif pairs_dict[RecTag] == "Repeated Correct Mode" :
		moveHand = 1
		pairs_dict[RecTag] = "Waiting for Green-card"



def cardDetection(msg, tag):
	global cardPosition
	global REDButtonAllert
	global YELLOWButtonAllert
	global NumOfGreenCard
	global correctPermission
	global ARRec
	global detected_buttons_pub

	red_card = "40"
	green_card = "39"
	yellow_card = "38"

	card_id = msg.data
	global red_counter
	if card_id == red_card:
		
		detected_buttons_pub.publish("RED Card Detected")

		if pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Correcting Mode"
			REDButtonAllert = True
			if correctPermission == False:
				correctPermission = True
			#correctPermission = True

		elif pairs_dict[tag] == "Waiting for Green-card":
			pairs_dict[tag] = "Repeat Correct Mode"
			#reac = 2
			#motionProxySelection(reac)

	elif card_id == yellow_card:
		detected_buttons_pub.publish("YELLOW Card Detected")

		if pairs_dict[ARRec] == "Right":
			pairs_dict[ARRec] = "Switch"
			print "SwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitch"


		if pairs_dict[tag] == False:
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Green-card":
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == "Corrected Mode":
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == True:
			pairs_dict[tag] = "Repeat Correct Mode"	


	elif card_id == green_card:
		detected_buttons_pub.publish("GREEN Card Detected")

		if pairs_dict[tag] == "Waiting for Green-card" :
			#red_counter = 0
			if correctPermission == True:
				pairs_dict[tag] = "Correct Happy Mode"

			elif correctPermission == "Partial Correction":
				pairs_dict[tag] = "Wrong Happy Mode"

		elif pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Wrong Happy Mode"
			REDButtonAllert = False
			#reac = 2
	cardPosition = 0

	#rospy.Subscriber('card_pose', Pose, cardPointedAt)


def ButtonDetectionR(msg, tag):

	pushed = msg

	global cardPosition
	global REDButtonAllert
	global YELLOWButtonAllert 
	global NumOfGreenCard
	global correctPermission
	global ARRec
	global detected_buttons_pub
	global red_counter
	
	if pushed:
		print "SwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitch"
		
		detected_buttons_pub.publish("RED Button Detected")
		print stop_state["FinishReading"]
		#rospy.loginfo("stop_state[FinishReading]")
		#rospy.loginfo(stop_state["FinishReading"])
		if pairs_dict[tag] == "Waiting for Red-card":
			#if stop_state["FinishReading"] == True:
			pairs_dict[tag] = "Correcting Mode"
			REDButtonAllert = True
			stop_state["StopReading"] = True
			if correctPermission == False:
				correctPermission = True
			#elif stop_state["FinishReading"] == False:
				#stop_state["FinishReading"] = False

		elif pairs_dict[tag] == "Waiting for Green-card":
			pairs_dict[tag] = "Repeat Correct Mode"



def ButtonDetectionG(msg, tag):

	pushed = msg

	global cardPosition
	global REDButtonAllert
	global NumOfGreenCard
	global correctPermission
	global ARRec
	global detected_buttons_pub
	global red_counter

	if pushed:
		detected_buttons_pub.publish("GREEN Button Detected")

		if pairs_dict[tag] == "Waiting for Green-card" :
			#red_counter = 0
			if correctPermission == True:
				pairs_dict[tag] = "Correct Happy Mode"

			elif correctPermission == "Partial Correction":
				pairs_dict[tag] = "Wrong Happy Mode"

		elif pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Wrong Happy Mode"
			REDButtonAllert = False
			if correctPermission == True:
				pairs_dict[tag] = "Correct Happy Mode"
	cardPosition = 0

	


def ButtonDetectionY(msg, tag):

	pushed = msg

	global cardPosition
	global REDButtonAllert
	global YELLOWButtonAllert
	global NumOfGreenCard
	global correctPermission
	global ARRec
	global detected_buttons_pub
	global red_counter

	if pushed:
		detected_buttons_pub.publish("YELLOW Button Detected")

		if pairs_dict[ARRec] == "Right":
			pairs_dict[ARRec] = "Switch"
			print "SwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitch"


		if pairs_dict[tag] == False:
			YELLOWButtonAllert = True
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Red-card":
			YELLOWButtonAllert = True
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Green-card":
			YELLOWButtonAllert = True
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == "Corrected Mode":
			YELLOWButtonAllert = True
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == True:
			YELLOWButtonAllert = True
			pairs_dict[tag] = "Repeat Correct Mode"	



def reactWithSpecificPoses(emotion, pose, pause, wordsAfter , factorSpeed = 1.0):

	global blinkThread
	#idleMovementMode("OFF")
	#blinkingModeOFF()
	#blinkThread.sleep()
	motionProxy.setBreathEnabled('Arms', False)

	emotionReaction = EYEEMOTIONS(proxy)

	times = changeSpeed(pose.times, factorSpeed)
	id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
	audioReactionProxy.post.playFile("/home/nao/audio/wav/monsterGrowl.wav")
	#story.post.say(wordsBefore)
	emotionReaction.set_emotion(emotion)
	#turn_on_eye()
	#time.sleep(pause)
	#motionProxy.wait(id,0)
	#time.sleep(1)
	#story.say(wordsAfter)
	#time.sleep(5)
	correctFlag = True
	pitch_angle = 0.3
	LookAtTheBook(pitch_angle)
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	#if wordsAfter != None:
	#story.say(wordsAfter)
	#blinkThread.start()
	#motionProxy.setBreathEnabled('Head', True)
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
	#idleMovementMode("ON")


def reactToBoredness(emotion, pose, pause, wordsAfter , factorSpeed = 1.0):

	global blinkThread
	#idleMovementMode("OFF")
	#blinkingModeOFF()
	#blinkThread.sleep()
	motionProxy.setBreathEnabled('Arms', False)

	emotionReaction = EYEEMOTIONS(proxy)

	times = changeSpeed(pose.times, factorSpeed)
	id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
	#story.post.say(wordsBefore)
	#emotionReaction.set_emotion(emotion)
	#turn_on_eye()
	#time.sleep(pause)
	#motionProxy.wait(id,0)
	#time.sleep(1)
	#story.say(wordsAfter)
	#time.sleep(5)
	correctFlag = True
	pitch_angle = 0.3
	LookAtTheBook(pitch_angle)
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	#if wordsAfter != None:
	story.say(wordsAfter)
	#blinkThread.start()
	#motionProxy.setBreathEnabled('Head', True)calculateWhereToPointAt
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStory(selectedStory, correctFlag)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
	#idleMovementMode("ON")


def reactToTheMistake( emotion, pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
	""" If the keys pressed are due to detection of mistake, the robot reacts.
		The motionProxy is a physical movement and certain words which shows robot's remorse

	"""
	global blinkThread
	global robot_state_pub
	#idleMovementMode("OFF")
	robot_state_pub.publish("Reaction Mode Started")
	blinkingModeOFF()
	idleMovementModeOFF()
	#blinkThread.sleep()
	motionProxy.setBreathEnabled('Arms', False)

	emotionReaction = EYEEMOTIONS(proxy)

	times = changeSpeed(pose.times, factorSpeed)
	id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
	story.post.say(wordsBefore)
	emotionReaction.set_emotion(emotion)
	turn_on_eye()
	#time.sleep(pause)
	#motionProxy.wait(id,0)
	#time.sleep(1)
	story.say(wordsAfter)
	#time.sleep(5)
	correctFlag = True
	pitch_angle = 0.3
	LookAtTheBook(pitch_angle)
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	robot_state_pub.publish("Reaction Mode Ended")
	#blinkThread.start()
	#motionProxy.setBreathEnabled('Head', True)
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStory(selectedStory, correctFlag)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
	blinkingModeON("ON")
	idleMovementModeON("ON")


def reactToTheMistake2( pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
	""" If the keys pressed are due to detection of mistake, the robot reacts.
		The motionProxy is a physical movement and certain words which shows robot's remorse

	"""
	global blinkThread
	#blinkThread.sleep()
	#idleMovementMode("OFF")
	blinkingModeOFF()
	idleMovementModeOFF()
	motionProxy.setBreathEnabled('Arms', False)

	emotionReaction = EYEEMOTIONS(proxy)

	times = changeSpeed(pose.times, factorSpeed)
	id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
	story.post.say(wordsBefore)
	emotionReaction.set_emotion("happy")
	turn_on_eye()
	#time.sleep(pause)
	#motionProxy.wait(id,0)
	#time.sleep(1)
	story.say(wordsAfter)
	#time.sleep(5)
	correctFlag = True
	pitch_angle = 0.3
	LookAtTheBook(pitch_angle)
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	#motionProxy.setBreathEnabled('Head', True)
	#postureProxy.goToPosture("Stand", 1.0)
	#readTheTaggedStory(selectedStory, correctFlag)
	#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
	blinkingModeON("ON")
	idleMovementModeON("ON")


def turn_on_eye():
	section1 = ["FaceLeds", "ChestLeds" ]
	proxy.createGroup("turn",section1)
	proxy.fadeRGB("turn", 0x00FFFFFF, 0.3)


def changeSpeed(times, factor):
	""" It changes the speed of predefined times for each pose movement

	"""

	for i in xrange(len(times)):
		times[i] = [x / float(factor) for x in times[i]]

	return times


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

global skip
skip = 0 

def idleMovementModeON(mode):
	global skip
	global idleThread
	#idleThread = threading.Timer(45, idleMovementModeON, [mode])
	#animationSelection = MOTION_ANIMATION_SELECTION()
	#idleThread.start()
	skip += 1
	#print k
	#if mode == "ON":		
		#emotionReaction.blink_eyes()
		#if skip >= 2:
			#global robot_state_pub
			#robot_state_pub.publish("Idle Mode Reaction Started") 
			#motionProxy.setBreathEnabled('Arms', False)
			#animationSelection.reactionIdleMovement()
			#motionProxy.setBreathEnabled('Arms', True)
			#robot_state_pub.publish("Idle Mode Reaction Ended") 
		


def idleMovementModeOFF():
	global idleThread
	global skip
	#emotionReaction.turn_off_eye()
	#idleThread.cancel()
	skip = 0 


def twoPageModeON(mode):
	global k
	global twoPageThread
	twoPageThread = threading.Timer(40, idleMovementModeON, [mode])
	animationSelection = MOTION_ANIMATION_SELECTION()
	idleThread.start()
	k += 1
	#print k
	if mode == "ON":		
		#emotionReaction.blink_eyes()
		if k >= 2: 
			motionProxy.setBreathEnabled('Arms', False)
			animationSelection.reactionIdleMovement()
			motionProxy.setBreathEnabled('Arms', True)
		


def twoPageModeOFF():
	global idleThread
	global k
	#emotionReaction.turn_off_eye()
	idleThread.cancel()
	k = 0 
	


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


def LookAtTheBook(pitch_angle, yaw_angle=0):
	""" Move the robot's head to look at the camera

	"""
	global fractionMaxSpeed
	motionProxy.setStiffnesses("Head", 1.0)
	pitch_angle = 0.2
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






def taskSelection():
	global NumOfGreenCard
	global FinishSign
	#print "NumOfGreenCard"
	#print NumOfGreenCard

	taskLevel_dict['PrevLevel'] = taskLevel_dict['CurrLevel'] 

	if NumOfGreenCard == 0:
		if taskLevel_dict['PrevLevel'] == "TaskNONE":
			taskLevel_dict['CurrLevel'] = "TaskONE"

	if NumOfGreenCard == 10:
		NumOfGreenCard = 0
		if taskLevel_dict['PrevLevel'] == "TaskNONE":
			taskLevel_dict['CurrLevel'] = "TaskONE"

		#elif taskLevel_dict['PrevLevel'] == "TaskONE":
			#taskLevel_dict['CurrLevel'] = "TaskTWO"

		#elif taskLevel_dict['PrevLevel'] == "TaskONE":
			#taskLevel_dict['CurrLevel'] = "TaskTHREE"

		elif taskLevel_dict['PrevLevel'] == "TaskTHREE":
			taskLevel_dict['CurrLevel'] = "TaskFINISH"

	if FinishSign == True:
		NumOfGreenCard = 0

		if taskLevel_dict['PrevLevel'] == "TaskONE":
			taskLevel_dict['CurrLevel'] = "TaskFINISH"

	if taskLevel_dict['PrevLevel'] != taskLevel_dict['CurrLevel']:
		global robot_state_pub
		robot_state_pub.publish( taskLevel_dict['CurrLevel'] + " Selected")

#def restartCoReader():


def TurnOffCoReader():

	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', False)
	motionProxy.killAll()


def GoToFinishLine(once, childName):

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
		
		
		story.setLanguage('English')
		story.say("\\rspd=80\\ hey " + childName)
		story.say("\\rspd=80\\ why don't you reed the rest of the book for me?")
		story.say("\\rspd=80\\ I really liked it when you reed it for me last time")
		motionProxy.setBreathEnabled('Arms', True)


def IntroduceNao(numOfStart, childName):
	"""
	Nao starts introducing itself when the book cover is in front of him 
	"""
	global robot_state_pub
	global blinkThread 
	facesize = 0.1

	robot_state_pub.publish("Introduce Nao Started")
	# First, wake up
	#motionProxy.wakeUp()
	postureProxy.goToPosture("Crouch", 0.5)

	if numOfStart == "1":
		LookAtTheBook(0.3, -0.8)
		faceTrackingStarted(facesize)
	turn_on_eye()
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	blinkingModeON("ON")
	#idleMovementModeON("ON")
	mode = "ON"

	#motionProxy.setBreathEnabled('Head', True)
	#motionProxy.rest()


	story.setLanguage('English')
	if numOfStart == "1":
		script =   ("\\rspd=90\\ Hello " + childName + "\\pau=700\\ Welcome back \\pau=500\\ "
					"\\rspd=90\\ Do you like to continue reading with me? \\pau=500\\"
					"\\rspd=90\\ Let's finish the book and see what happens in the end \\pau=500\\"
					"\\rspd=90\\ I'm so excited")
	else:
		script = ("\\rspd=90\\ ")
	story.say(script)
	#story.say("\\rspd=90\\ Hello everyone \\pau=700\\ My name is Nao \\pau=500\\ I really like reading short stories")
	#story.say("\\rspd=90\\ Do you want to listen to them?")
	#story.say("\\rspd=90\\ sometimes I make mistakes, can you help me to correct them?")
	#time.sleep(0.1)
	#story.say("\\rspd=90\\ If you want to read with me, please bring the book")
	#story.say("\\rspd=90\\ and don't forget the red, green and yellow card")
	#story.say("\\rspd=90\\ \\pau=50\\ you can show me the red card if I make a mistake")
	#story.say("\\rspd=90\\ \\pau=50\\ the green card when I'm correct")
	#story.say("\\rspd=90\\ \\pau=50\\ and the yellow one, when you want me to repeat")
	#story.say("\\rspd=90\\ Hello")
	robot_instruction_pub.publish(script)
	pitch_angle = 0.2
	#LookAtTheBook(pitch_angle)
	time.sleep(0.5)
	idleMovementModeON("ON")
	robot_state_pub.publish("Introduce Nao Ended")


def removeTheTag(tag, storyContent):
	""" Find and remove the tag given to the function and leave the word connected to them intact

	"""

	while True:
		foundTag = re.search(tag, storyContent)
		if foundTag == None:
			break
		storyContent = storyContent[:foundTag.start(0)] + storyContent[foundTag.end(0):]

	return storyContent



def main():

	# Initiate the node that does the main computing
	rospy.init_node('read_lines_location_event')

	# Inititate the publishers
	global robot_state_pub
	robot_state_pub = rospy.Publisher("robot_state", String, queue_size=5)

	global child_name_pub
	child_name_pub = rospy.Publisher("child_name", String, queue_size=5)

	global enabled_state_pub
	enabled_state_pub = rospy.Publisher("enabled_state", String, queue_size=5)

	global robot_instruction_pub
	robot_instruction_pub = rospy.Publisher("instruction_state", String, queue_size=100)

	global robot_gesture_pub
	robot_gesture_pub = rospy.Publisher("gesture_state", String, queue_size=5)

	global page_state_pub
	page_state_pub = rospy.Publisher("page_state", String, queue_size=5)

	global correction_state_pub
	correction_state_pub = rospy.Publisher("correction_state", String, queue_size=5)

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


	# Define Nao IP and inititate all yhe Nao related API
	#nao_IP = rospy.get_param('~nao_ip')
	nao_IP = '192.168.1.64'
	

	myBroker = ALBroker("myBroker",        
		"0.0.0.0",   # listen to anyone
    	0,           # find a free port and use it
    	nao_IP,         # parent broker IP
    	9559)       # parent broker port)

	#myBroker.shutdown()	


	global story
	story = ALProxy("ALTextToSpeech", nao_IP, 9559)
	
	#global conversration
	#conversation = ALProxy("ALTextToSpeech", nao_IP, 9559)

	global motionProxy
	motionProxy = ALProxy("ALMotion", nao_IP, 9559)

	global postureProxy
	postureProxy = ALProxy("ALRobotPosture", nao_IP, 9559)
	
	global audiomotionProxy
	audiomotionProxy = ALProxy("ALAudioPlayer", nao_IP, 9559)
	
	global tracker
	tracker = ALProxy("ALTracker", nao_IP, 9559)

	global proxy
	proxy = ALProxy("ALLeds", nao_IP, 9559)

	global memory
	memory = ALProxy("ALMemory", nao_IP, 9559)

	global audioReactionProxy
	audioReactionProxy = ALProxy("ALAudioPlayer", nao_IP, 9559)


	global SpeechCondition 
	SpeechCondition = SpeechModule("SpeechCondition")

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

	robot_instruction_pub.publish(instructionText)

	childName = initialization_dict["ChildName"]
	child_name_pub.publish(childName)

	robot_speed_pub.publish(str(initialization_dict["MoveHandSpeed"]) + "with" + str(initialization_dict["MoveHandSpeed"]) + "delay")
	# Face tracking activated
	faceTrackingEnded()
	enabled_state_pub.publish("Face Tracking is OFF")
	facesize = 0.1
	global restingEnabled 
	restingEnabled = False
	faceTrackingStarted(facesize)
	enabled_state_pub.publish("Face Tracking is ON")
	
	global space
	space      = motion.FRAME_ROBOT

	# Select a story and activity level
	correctFlag = False
	activityLevel = rospy.get_param('actlevel', 'medium')
	#mistakeNum = numberOfMistakesSelection(activityLevel)









	# The function to introduce Nao and stop the face tracking 
	time.sleep(0.5)
	IntroduceNao(numOfStart, childName)
	faceTrackingEnded()
	enabled_state_pub.publish("Face Tracking is OFF")

	# Make the robot to look at the text
	pitch_angle = 0.3
	yaw_angle = 0.8
	LookAtTheBook(pitch_angle)
	

	# Subscribe to the tags
	rospy.Subscriber('tag_id_state', String, tagDetection)

	
	



if __name__ == "__main__":

	global story
	
	try:
		main()
		rospy.spin()

    	except rospy.ROSInterruptException:
        	print
        	print "Interrupted by user, shutting down"
        	#story.stopAll()
        	rospy.loginfo("AR Tag Tracker node terminated.")
        	myBroker.shutdown()
        	sys.exit(0)

