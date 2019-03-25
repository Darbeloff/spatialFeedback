#!/usr/bin/env python
import rospy
import numpy as np
import math
import random
import matplotlib.mlab as mlab
import time
from std_msgs.msg import Float32

def start():
	# initialize node
	rospy.init_node('compDistVel', anonymous = True)

	# initialize lead arm publisher
	global compVelPub
	compVelPub = rospy.Publisher('compVelPub', Float32, queue_size = 10)

	global testPub
	testPub = rospy.Publisher('testPub', Float32, queue_size = 10)

	# generate computer velocity disturbances
	compVelPubRate = 100.0; # Hz
	compVelVector = genCompVelDisturbances(compVelPubRate)	

	# just so we can see start and stop
	time.sleep(3)

	# begin sending traj
	sendDisturbances(compVelPubRate, compVelVector)

	# rospy.spin()
	rospy.spin()

def genCompVelDisturbances(compVelPubRate):	
	# random seed
	random.seed()

	# decide on number of disturbances
	maxDisturb = 5.0
	minDisturb = 5.0
	numDisturbances = int(round(random.random()*(maxDisturb-minDisturb) + minDisturb))

	trialLength = 20.0 # seconds

	# create list to divide trial into parts
	trialPartList = range(0,int(maxDisturb))
	trialPartLength = trialLength/maxDisturb
	trialPartList = [i*trialPartLength for i in trialPartList]

	# length of disturbances
	# maxLength = 3 # seconds
	maxLength = rospy.get_param("distTime")
	minLength = maxLength 

	# amp of disturbances
	# maxAmp = 13 # deg/sec
	maxAmp = rospy.get_param("distAmp")
	minAmp = maxAmp

	# create disturbances
	distAll = []
	for i in range(0,numDisturbances):
		lengthDisturb = random.random()*(maxLength-minLength)+minLength

		# starting times are chosen by dividing the trial into maxDisturb equal parts. One of 			the parts is chosen at random and then the startTime occurs somewhere within that 			part while ensuring that it ends prior to that part being over. 		
		trialPart = random.choice(trialPartList) # random part of trial chosen
		trialPartList.remove(trialPart) # remove that from the list so it cant be used again
		# now find start time and amplitude for disturbance
		startDisturb = random.random()*(trialPartLength - lengthDisturb) + trialPart
		ampDisturb = random.random()*(maxAmp-minAmp)+minAmp

		# should be either -1 or 1
		dirDisturb = int(round(random.random()))
		if dirDisturb == 0:
			dirDisturb = -1

		# append to the full disturbance list
		distAll.append([startDisturb, lengthDisturb, ampDisturb, dirDisturb])
		
		# sort it by start times so when we concatenate them it works
		distAll.sort(key=lambda x: x[0])

	# now create full 20 second trajectory
	compVelVector = []
	timeVecLookup = np.linspace(0,trialLength,trialLength*compVelPubRate)
	for i in range(0,len(distAll)):
		sigma = distAll[i][1]/6 # roughly
		x = np.linspace(0 - 3*sigma, 0 + 3*sigma, distAll[i][1]*compVelPubRate)
		distGaussian = mlab.normpdf(x,0,sigma)
		distGaussian = distGaussian*distAll[i][2]*distAll[i][3]
	
		# now replace the 0s in the compVelVector with the distGaussian starting at startDist
		# first identify starting point in compVelVector (match index to time) 
		startIndex = timeToIndex(distAll[i][0], timeVecLookup)
		endIndex = startIndex+len(distGaussian)
		compVelVector.extend([0]*int(startIndex-trialPartLength*compVelPubRate*i))
		compVelVector.extend(distGaussian)
		compVelVector.extend([0]*int(trialPartLength*compVelPubRate-endIndex))

	return compVelVector

def timeToIndex(time,timeVecLookup):
	closestIndex = (np.abs(timeVecLookup-time)).argmin()
	return closestIndex

def sendDisturbances(compVelPubRate, compVelVector):
	# ask lead arm to follow randomized trajectory
	loopNum = 0
	rate = rospy.Rate(compVelPubRate) # hertz
	while (not rospy.is_shutdown()) and (loopNum < (len(compVelVector)-1)):
		# do stuff
		loopNum = loopNum + 1
		compVelPub.publish(compVelVector[loopNum])

		# sleep to maintain loop rate
		rate.sleep()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
