"""
with sample of python documentation
"""

from naoqi import *
import time
check = 0


# create python module
class myModule(ALModule):
  """python class myModule test auto documentation: comment needed to create a new python module"""


  def pythondatachanged(self, strVarName, value):
    """callback when data change"""
    print "datachanged", strVarName, " ", value, " "
    global check
    check = 1

  def _pythonPrivateMethod(self, param1, param2, param3):
    global check

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
    print "SpeechDone"
    print SpeechDone


broker = ALBroker("pythonBroker","0.0.0.0",9999,"nao.local",9559)


# call method
try:

  pythonModule = myModule("pythonModule")
  prox = ALProxy("ALMemory")

  story = ALProxy("ALTextToSpeech", "nao.local", 9559)
  story.say("hello everyone")

  #prox.insertData("val",1) # forbidden, data is optimized and doesn't manage callback
  prox.subscribeToEvent("ALTextToSpeech/TextDone","pythonModule", "onTextDone") #  event is case sensitive !
  global SpeechDone

  if SpeechDone:
    print SpeechDone


except Exception,e:
  print "error"
  print e
  exit(1)

while (1):
  time.sleep(2)
