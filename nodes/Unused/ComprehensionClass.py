from naoqi import ALProxy 
import codecs
import time
import re
import random
import rospy


class COMPREHENSIONTASK:
def ComprehensionTask():
	CompType = CompType_dict["ambiguous"]

	if CompType == 1:
		AmbigiousBehaviour()

	elif CompType == 2:
		EgocentricBehaviour()

	elif CompType == 3:
		AddresseeBehaviour()


def AmbigiousBehaviour():
	story.post.say("\\rspd=90\\ can you move two beads to the right?")


def EgocentricBehaviour():
	story.post.say("\\rspd=90\\ can you move two beads to my right?")

def AddresseeBehaviour():
	story.post.say("\\rspd=90\\ can you move two beads to your right?")