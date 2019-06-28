# -*- encoding: UTF-8 -*- 

'''Cartesian control: Arm trajectory example'''
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
import animations.winner_seated_pose_left_arm
import animations.winner_seated_pose_right_arm

import sys
import motion
import almath
from naoqi import ALProxy


def StiffnessOn(proxy):
  #We use the "Body" name to signify the collection of all joints
  pNames = "Body"
  pStiffnessLists = 1.0
  pTimeLists = 1.0
  proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def changeSpeed(times, factor):
    """ It changes the speed of predefined times for each pose movement

    """

    for i in xrange(len(times)):
        times[i] = [x / float(factor) for x in times[i]]

    return times

def main(robotIP):
    ''' Example showing a path of two positions
    Warning: Needs a PoseInit before executing
    '''

    # Init proxies.
    #global motionProxy
    #motionProxy = ALProxy("ALMotion", nao_IP, 9559)
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)

    # Send NAO to Pose Init
    postureProxy.goToPosture("Crouch", 0.3)

    effector   = "LArm"
    space      = motion.FRAME_ROBOT
    axisMask   = almath.AXIS_MASK_VEL    # just control position
    isAbsolute = False

    # Since we are in relative, the current position is zero
    currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Define the changes relative to the current position
    dx         =  0.03      # translation axis X (meters)
    dy         =  0.03      # translation axis Y (meters)
    dz         =  0.00      # translation axis Z (meters)
    dwx        =  0.00      # rotation axis X (radians)
    dwy        =  0.00      # rotation axis Y (radians)
    dwz        =  0.00      # rotation axis Z (radians)
    targetPos  = [dx, dy, dz, dwx, dwy, dwz]

    # Go to the target and back again
    path       = [targetPos, currentPos]
    times      = [2.0, 4.0] # seconds

    #motionProxy.positionInterpolation(effector, space, path,
    #                                  axisMask, times, isAbsolute)
    sleepTime = 1
    factorSpeed = 0.4
    pose = animations.winner_seated_pose
    #pose = animations.winner2_seated_pose
    pose = animations.winner_seated_pose_right_arm
    pose = animations.winner_seated_pose_left_arm

    times = changeSpeed(pose.times, factorSpeed)
    id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)


if __name__ == "__main__":
    #robotIp = 'nao.local'
    robotIp = '192.168.1.64'

    if len(sys.argv) <= 1:
        print "Usage python motion_cartesianArm1.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
