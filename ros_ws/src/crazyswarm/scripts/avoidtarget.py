#!/usr/bin/env python
'''
avoid target script
- identifies target cf
- sends other cfs to random heights or to static formation
- loads path for targetCF (trajectory or teleop) 
- loops to check current position of targetCF relative to other present CFs
- adjusts accordingly to avoid collisions 
'''


#import different libraries

import numpy as np #python computational library
import time
import array
import rospy
import random

from pycrazyswarm import * #crazyswarm 
import uav_trajectory #trajectory classes
from waypoints import Waypoint #waypoint class

#for formation coordinates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import yaml
import os

#for hungarian algorithm
from scipy.spatial import distance

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
cfs = allcfs.crazyflies
trajectory = uav_trajectory.Trajectory()
breathe = 1.0
step = 0.1
spd = 0.2
filepath = '/home/trailCrazyswarm/crazyswarm/ros_ws/src/crazyswarm/scripts'

print "Which CF is the target? (CFbyId)"
targetCFid = int(raw_input())
targetCF = allcfs.crazyfliesById[targetCFid]


def main():
	#target Input

	
	targetCF.takeoff(targetHeight=1.0, duration=3.0)
	timeHelper.sleep(3.0)
	
	toStart = raw_input("Send target to start? (y/n)")
	
	if toStart == 'y':
		targetCF.goTo(np.array([2.0, -2.0, 1.0]), 0, 3.0)
		timeHelper.sleep(3.0)
		
	toRand = raw_input("Send CFs to random heights? (y/n/h(hover))")
	
	if toRand == 'y':
		randHeights()
	elif toRand == 'h':
		hoverCallback()
	
	timeHelper.sleep(5.0)
	
	targetflight = raw_input("Generate trajectory or control manually? (traj/tele)")	
	
	if targetflight == 'traj':
		trajPath()
	elif targetflight == 'tele':
		manualTeleop()
	

def manualTeleop():
	#TODO: There has to be a more elegant way to do this :((
	#Try using turtle teleop_twist? (figure out how to incorporate while still maintaining the loop)
	while True:
		flyDir = raw_input("Flight direction:")
		
		if flyDir == 'w':
			targetCF.goTo((np.array(targetCF.position()) + np.array([step, 0, 0])), 0, spd)
		elif flyDir == 's':
			targetCF.goTo((np.array(targetCF.position()) - np.array([step, 0, 0])), 0, spd)
		elif flyDir == 'a':
			targetCF.goTo((np.array(targetCF.position()) + np.array([0, step, 0])), 0, spd)
		elif flyDir == 'd':
			targetCF.goTo((np.array(targetCF.position()) - np.array([0, step, 0])), 0, spd)
		elif flyDir == 'e':
			targetCF.goTo((np.array(targetCF.position()) + np.array([0, 0, step])), 0, spd)
		elif flyDir == 'r':
			targetCF.goTo((np.array(targetCF.position()) - np.array([0, 0, step])), 0, spd)
		elif flyDir == 'land':
			targetCF.land(targetHeight=0.003, duration=4.0)
			break
		else:
			continue
		
		distanceCheck()
		timeHelper.sleep(0.5)
		
def distanceCheck():
	
	#make an array/matrix with all the current positions
	#go through each value (by x, y, z) 
	#make sure nothing is within a 0.3 range ?
	currentPositionMatrix = []
	
	for cf in cfs:
		currentPositionMatrix.append(cf.position())
	
	print currentPositionMatrix
	
	i = 0
	j = 0	
	for cf in cfs: #baseCF row i
		basecf = cf.id
		for cf in cfs: #comparedCF row j
			current
			if cf.id == basecf: #skips itself
				continue
			else:
				dist = distance.euclidean(np.array(allcfs.crazyfliesById[basecf].position()), np.array(cf.position()))
				
				if dist < 0.3:
					##TODO: if they're too close, adjust one to a certain distance away
					##figure out how to come up with that?
					#diffx = currentPositionMatrix[i][0] - currentPositionMatrix[j][0] 
					###error: list index out of range?? figure that out pls
					#diffy = currentPositionMatrix[i][1] - currentPositionMatrix[j][1] 
					#diffz = currentPositionMatrix[i][2] - currentPositionMatrix[j][2]
					#print diffx, diffy, diffz
					print 'CF',basecf,"is too close to CF",cf.id
					cf.goTo((np.array(cf.position()) + np.array([- step, - step, - step])), 0, 0.5)
					#cf is referring to both base and current?? 
				
			j = j + 1
		currentPositionMatrix.append(cf.position())
		i = i + 1
	
	
def randHeights():
	rand_heights = None

	if len(cfs) > 1:
		rand_heights = [random.random() for cf in cfs]
		lowest = min(rand_heights)
		highest = max(rand_heights)

		# ensure we fill up the full range of heights
		MIN_HEIGHT = 0.6
		MAX_HEIGHT = 1.6
		scale = (MAX_HEIGHT - MIN_HEIGHT) / (highest - lowest)

		for i in range(len(rand_heights)):
			rand_heights[i] = scale * (rand_heights[i] - lowest) + MIN_HEIGHT
	else:
		rand_heights = [1.0]

	heights = {}
	for cf, height in zip(cfs, rand_heights):
		heights[cf] = height
		
	for cf in allcfs.crazyflies:
		startHoverMatrix = np.array(cf.initialPosition) + np.array([0, 0, heights[cf]])
		cf.goTo(startHoverMatrix, 0, 5.0)

	timeHelper.sleep(5.0) 

def hoverCallback():	
	allcfs.takeoff(targetHeight=1.0, duration=3.0)
	timeHelper.sleep(3.0)
	
def trajPath():
	timeHelper.sleep(2.0)

if __name__ == "__main__":
	main() 
