#!/usr/bin/env python
'''
avoid target script
- identifies target cf
- sends other cfs to random heights or to static formation
- loads path for targetCF (trajectory or teleop) 
- loops to check current position of targetCF relative to other present CFs
- adjusts accordingly to avoid collisions 

#movements are controlled by cf_teleop.py imported from scripts folder
# using input in distance and direction

**reference for code is in avoidTarget-Crazyswarm folder/package
	- ros_ws/src/crazyswarm/scripts/avoidtarget.py (really only used the fly to random heights part)
	- crazyflie-firmware/src/modules/src/avoidtarget.c (has the logic/math of determining goal positions to adjust from near target)
									(but computations are questionable and weird)

TODO: Rewrite to work with ROS so that individual nodes can work the different parts of the code simultaneously
- TargetCF tracking: to check distances between each cf and the target and determine when they're too close and adjust back
- AllCF tracking: to check distances between all of the cfs and make sure they don't get too close to each other either
- Movement and path generation: uses matlab trajectory generation codes to make smooth movements for each cf as it 
  receives new goal positions (to make avoiding look more organic)

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

##for formation coordinates
#from geometry_msgs.msg import Pose
#from geometry_msgs.msg import PoseArray
#import yaml
#import os

#for euclidean distance
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

		timeHelper.sleep(0.5)
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
			
def distanceCheck():
	
	#make an array/matrix with all the current positions
	#go through each value (by x, y, z) 
	#make sure nothing is within a 0.3 range
	'''
	#### Just some issues ####
	
	first checks loop compares all cfs to target cf 
	second checks loop compares all cfs to all cfs (not including target assuming it should already be out of the way) 
		two for loops one after another isn't checking both one after the other for some reason?
		or it is just the adjustments are happening at a weird pace because once the others adjust
	they end up coming back towards the target to avoid the other cfs (and end up too near again)
	 but because of loop sequences, they don't adjust back until the next command is provided
	
	TODO: Do this on ROS so that they can run as individual nodes simultaneously
	
	This is all a bit weird right now and doesn't necessarily work too well lol 
	it looks like it works ish but the desire_disp equation isn't being used at all because the values it generates are ridiculously large
	so it's never smaller than the maxdisp but if the maxdisp were bigger it would be impractical for the movements of the cfs
	need to find a different equation for desired_disp that will generate slightly larger numbers when the dist is very small 
	but relatively smaller numbers when the dist is bigger (but still smaller than the maintaining distance) 
	
	Also because the loops are separate and done one after the other and go through the CFs in numerical order
	It doesn't do a cross check if the distances are too near after adjusting for the previous movement of the target
	so if one cf adjusted because it was too near another cf but by doing that ended up near another cf that came before it 
	in the checking sequence, it wouldn't know that it was too near that later one
	
	It starts working better once everything is already spread out because the starting grid is already pretty tight
	it doesn't pick up on how near they are after the first movement which pushes back and affects the reactions the rest of the way
	until a more spread formation is created then they start behaving more ideally
	'''
	maxdisp = 0.4
	
	for cf in cfs:
		othercf = cf.id 
		if othercf != targetCFid:
			dist = distance.euclidean(np.array(targetCF.position()), np.array(allcfs.crazyfliesById[othercf].position()))
			if dist <= 0.4:
				#print 'CF',othercf,"is too close to target", dist
				cfpos = np.array(cf.position())
				tarpos = np.array(targetCF.position())
				delta = cfpos - tarpos
				delta_unit = delta / (dist)
				desired_disp = 0.15 / (dist + m.pow(dist, 2))
				disp = np.min([desired_disp, maxdisp])
				print 'target to', othercf, disp
				allcfs.crazyfliesById[othercf].goTo(np.array(cf.position()) + np.array(disp * delta_unit), 0, spd)
			else: ##maybe reverse other and base??? hmmmm ToTry 
				for cf in cfs: #baseCF row i
					basecf = cf.id
					if othercf != basecf and basecf != targetCFid: #skips itself and target
						otherdist = distance.euclidean(np.array(allcfs.crazyfliesById[basecf].position()), np.array(allcfs.crazyfliesById[othercf].position()))
						if otherdist < 0.4:
							#print 'CF', basecf, 'is too close to CF', currentcf
							currpos = np.array(allcfs.crazyfliesById[othercf].position())
							basepos = np.array(allcfs.crazyfliesById[basecf].position())
							delta = currpos - basepos
							delta_unit = delta / otherdist
							#print delta_unit
							desired_disp = 0.15 / (otherdist + m.pow(otherdist, 2))
							disp = np.min([desired_disp, maxdisp])
							#print desired_disp
							print 'cf', othercf, basecf, disp
							allcfs.crazyfliesById[othercf].goTo(currpos + np.array(disp * delta_unit), 0, spd)
			
		


def randHeights():
	##something isn't working here anymore????
	## also not working in CFdemo script
	# something changed somewhere?????? figure that out ugh
	### my dumbass just forgot to make them takeoff instead of goTo :<<<
	rand_heights = None
	
	
	if int(len(cfs)) > 1:
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
	
	print rand_heights

	heights = {}
	for cf, height in zip(cfs, rand_heights):
		heights[cf] = height
		
	for cf in cfs:
		startHoverMatrix = np.array(cf.initialPosition) + np.array([0, 0, heights[cf]])
		cf.takeoff(targetHeight=heights[cf], duration=3.0)

	timeHelper.sleep(3.0) 

def hoverCallback():	
	allcfs.takeoff(targetHeight=1.0, duration=3.0)
	timeHelper.sleep(3.0)
	
def trajPath():
	timeHelper.sleep(2.0)

if __name__ == "__main__":
	main() 
