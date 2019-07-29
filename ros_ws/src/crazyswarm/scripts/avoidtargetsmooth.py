#!/usr/bin/env python
'''
same as avoid target script
incorporates matlab trajectory generation 
'''


#import different libraries

import numpy as np #python computational library
import time
import array
import rospy
import random
import math as m

from pycrazyswarm import * #crazyswarm 
import uav_trajectory #trajectory classes
from waypoints import Waypoint #waypoint class
import cf_teleop

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
	
	targetflight = raw_input("Generate trajectory or control manually? (keys/info)")	
	
	if targetflight == 'info':
		inputTeleop()
	elif targetflight == 'keys':
		manualTeleop()
	
	allcfs.land(targetHeight = 0.003, duration=4.0)

def manualTeleop():
	#TODO: There has to be a more elegant way to do this :((
	#Try using turtle teleop_twist? (figure out how to incorporate while still maintaining the loop)
	while True:
		distanceCheck()
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

		timeHelper.sleep(0.5)
		
def inputTeleop():	
	while True:
		distanceCheck()
		print "Input info:"
		dist = float(raw_input("Distance:"))
		dirx = float(raw_input("Direction (in degrees):"))
		dura = float(raw_input("Duration:"))
		cftele = cf_teleop.Teleop(targetCFid, dist, dirx, dura)
		dx = cftele.dx()
		dy = cftele.dy()
		print dx, dy
		if dist >= 0:
			targetCF.goTo((np.array(targetCF.position()) + np.array([dx, dy, 0])), 0, dura)
		else:
			targetCF.goTo((np.array(targetCF.position()) + np.array([-dx, dy, 0])), 0, dura)
		timeHelper.sleep(dura)
		if dist == 'land':
			targetCF.land(targetHeight=0.003, duration=4.0)
			timeHelper.sleep(4.0)
			break
			
	###THIS doesnt work cuz even if yaw changes, there is not "forward" for the cf
	## it will always go to the specified location 

def distanceCheck():
	
	#make an array/matrix with all the current positions
	#go through each value (by x, y, z) 
	#make sure nothing is within a 0.3 range ?
	
	#first checks loop compares all cfs to target cf 
	maxdisp = 0.3 
	
	for cf in cfs:
		othercf = cf.id 
		if othercf != targetCFid:
			dist = distance.euclidean(np.array(targetCF.position()), np.array(allcfs.crazyfliesById[othercf].position()))
			if dist <= 0.3:
				print 'CF',othercf,"is too close to target", dist
				cfpos = np.array(cf.position())
				tarpos = np.array(targetCF.position())
				delta = cfpos - tarpos
				delta_unit = delta * (1.0 / dist)
				desired_disp = 1.5 / (dist + m.pow(dist, 2))
				disp = np.min([desired_disp, maxdisp])
				allcfs.crazyfliesById[othercf].goTo(np.array(cf.position()) + np.array(disp * delta_unit), 0, spd)
			else:
				continue		
		else:
			continue
	
	#second checks loop compares all cfs to all cfs (not including target assuming it should already be out of the way) 
		
	for cf in cfs: #baseCF row i
		basecf = cf.id
		for cf in cfs: #comparedCF row j
			currentcf = cf.id
			if currentcf != basecf and currentcf != targetCFid: #skips itself and target
				dist = distance.euclidean(np.array(allcfs.crazyfliesById[basecf].position()), np.array(allcfs.crazyfliesById[currentcf].position()))
				if dist <= 0.3:
					print 'CF', basecf, 'is too close to CF', currentcf
					currpos = np.array(allcfs.crazyfliesById[currentcf].position())
					basepos = np.array(allcfs.crazyfliesById[basecf].position())
					delta = currpos - basepos
					delta_unit = delta * (1.0 / dist)
					desired_disp = 1.5 / (dist + m.pow(dist, 2))
					disp = np.min([desired_disp, maxdisp])
					allcfs.crazyfliesById[currentcf].goTo(np.array(cf.position()) + np.array(disp * delta_unit), 0, spd)
			else:
				continue


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
		cf.goTo(startHoverMatrix, 0, 2.0)

	timeHelper.sleep(2.0) 

def hoverCallback():	
	allcfs.takeoff(targetHeight=1.0, duration=3.0)
	timeHelper.sleep(3.0)
	
def trajPath():
	timeHelper.sleep(2.0)

if __name__ == "__main__":
	main() 
