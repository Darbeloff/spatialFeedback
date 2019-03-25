#!/usr/bin/env python
import rospy
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

# define some global variables
def defineGlobalVars():
	global humVel
	humVel = 0
	global compVel
	compVel = 0
	global limVal
	limVal = 1
	global encVal
	encVal = 0
	global encPosOffset
	encPosOffset = 0

def humCallback(data):
	# max velocity
	maxHumVel = 13 # degrees per second?

	# make deadband because joy is annoying
	joyVel = data.axes[1]
	if abs(data.axes[1]) < .1:
		joyVel = 0

	global humVel
	humVel = float(maxHumVel*joyVel)

def compCallback(data):
	# get and assign computer velocity to global
	global compVel
	compVel = float(data.data)

def ardCallback(data):
	# just read in the limit switches
	global limVal
	limVal = float(data.data[1])

	global encVal
	encVal = float(data.data[0])
	correctedEncPub.publish(encVal - encPosOffset)

def start():
	# initialize node
	rospy.init_node('combinedVelToArduino', anonymous = True)

	# initialize xbox2Glasses publisher
	global vel2ArduinoPub
	vel2ArduinoPub = rospy.Publisher('vel2ArduinoPub', Float32, queue_size = 10)

	# initalize encoder publisher
	global correctedEncPub
	correctedEncPub = rospy.Publisher('correctedEncPub', Float32, queue_size = 10)

	# subscribe to joy, our human command source
	humSubscriber = rospy.Subscriber("joy", Joy, humCallback)

	# subscribe to computer velocity generator
	compSubscriber = rospy.Subscriber('compVelPub', Float32, compCallback)

	# subscribe to arduino limit switch
	compSubscriber = rospy.Subscriber('ard2ControlPub', Float32MultiArray, ardCallback)

	# calibrate weight position
	calibrateWeight()

	# go to send command loop
	sendCommand()

	# rospy.spin()
	rospy.spin()

def sendCommand():
	# send send send
	pubRate = 50 # hertz
	rate = rospy.Rate(pubRate) # hertz

	# number of times through loop we go backwards
	limNumTimes = 15
	limCounter = 0
	limTrig = 0

	while (not rospy.is_shutdown()):
		# first average signals
		#compVel = 0 # just for now
		commandSignal = (compVel + humVel)/2.0

		# combine it with the limit switch...if limVal = 0, multiply by -1, else stay with 1;
		if limVal == 0:
			limTrig = 1 # start modifying commandSignal

		if limTrig == 1 and limNumTimes >= limCounter: # actually do the flipping
			if encVal - encPosOffset >= 0:
				commandSignal = 5
			else:
				commandSignal = -5
			limCounter = limCounter + 1
		elif limNumTimes < limCounter: # reset
			limCounter = 0
			limTrig = 0

		# iterate loop number
		# loopNum = loopNum + 1

		# publish that signal
		vel2ArduinoPub.publish(commandSignal)

		# sleep to maintain loop rate
		rate.sleep()

def calibrateWeight():
	pubRate = 50 # hertz
	rate = rospy.Rate(pubRate) # hertz
	# go until we hit limit switch
	while (not rospy.is_shutdown()):
		if limVal == 0:
			humVelDir = np.sign(humVel)
			break

		vel2ArduinoPub.publish(humVel/2.0)
		rate.sleep()

	encPos1 = encVal
	# correctedEncPub.publish(encVal - encPosOffset)

	# get off limit switch
	while (not rospy.is_shutdown()):
		if limVal != 0:
			break
		vel2ArduinoPub.publish(humVel/2.0)
		rate.sleep()

	# now move toward second limit switch
	while (not rospy.is_shutdown()):
		if limVal == 0:
			break
		vel2ArduinoPub.publish(humVel/2.0)
		rate.sleep()

	encPos2 = encVal

	# get offset
	global encPosOffset
	encPosOffset = (encPos1 + encPos2)/2

	# finally move to center
	while (not rospy.is_shutdown()):
		if abs(encVal - encPosOffset) < 20:
			# correctedEncPub.publish(encVal - encPosOffset)
			break
		vel2ArduinoPub.publish(3.0*humVelDir)
		rate.sleep()

	return

if __name__ == '__main__':
    try:
	defineGlobalVars()
	start()
    except rospy.ROSInterruptException:
        pass
