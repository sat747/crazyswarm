#!/usr/bin/env python

"""
Single demo python file that runs through different 
flight configurations for lab demos

Configurations:
 - Static formations (cfs form single shapes)
	- input: yaml file with Drone_Coordinates data points
 - Dynamic trajectories (swarm moves together following a trajectory pattern)
	- input: csv trajectory file (generated thru matlab)
 - Waypoints (each cf is assigned multiple points to fly to) 
	- input: csv waypoint file (TODO: generated thru matlab?? make waypoint file from user input entirely)
 - *Interactive: Follow (swarm will fly towards a certain target, maintaining a safe flight distance)
 - *Interactive: Avoid (swarm will adjust and avoid as a target moves within range)

*doesn't work yet

NOTES:
** make sure all files for execution (eg. yaml, csv) are within the appropriate folders
	- 'coordfiles' - yaml files with Drone_Coordinates
	- 'trajfiles' - csv files with trajectory coordinates
	- 'wayfiles' - csv files with waypoints coordinates and CF ids 
	
** if executing multiple flight configurations consecutively,
	ensure that the number of needed active crazyflies are consistent
	(or else program will crash and potentially shutdown cfs midflight)
	
** each function/configuration (TODO) exists as an independent .py file
	in the scripts folder for simpler executions and easier editing, updating or debugging
	- specific comments on individual parts and functions can be found in the separate files
	- Static: TRAIL_listener,_talker,_multilistener,_multitalker.py
	- Dynamic: figure8_csv.py (og crazyswarm), trajectorygen.py
	- Waypoints: waypoints.py (og cs), modwaypoints.py 
	- Avoid: avoidtarget.py, avoid_trajgen.py
	
** input 'stop' to emergency stop the program
	
"""

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
from scipy.optimize import linear_sum_assignment

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
trajectory = uav_trajectory.Trajectory()
breathe = 1.0
filepath = '/home/trailCrazyswarm/crazyswarm/ros_ws/src/crazyswarm/scripts'


def main():
		
	while True:
		print("What do you want to do?")
		print("Choose: takeoff, land, hover, static, dynamic, waypoints, goto, (follow, or avoid)")
		task = raw_input()
		
		if task == 'takeoff':
			takeoffCallback() 
			timeHelper.sleep(breathe)
		elif task == 'land':
			landCallback()
			timeHelper.sleep(breathe)
			break
		elif task == 'hover':
			hoverCallback()
			timeHelper.sleep(breathe)
		elif task == 'static':
			static()
			timeHelper.sleep(breathe)
		elif task == 'dynamic':
			dynamic()
			timeHelper.sleep(breathe)
		elif task == 'waypoints':
			waypoints()
			timeHelper.sleep(breathe)
		elif task == 'follow':
			follow()
			timeHelper.sleep(breathe)
		elif task == 'avoid':
			avoid()
			timeHelper.sleep(breathe)
		elif task == 'stop':
			timeHelper.sleep(breathe)
			allcfs.emergency()
			break
		else:
			print("Invalid input, choose again")
			continue
			
	

def takeoffCallback():	
	tarHeight = 0.5
	allcfs.takeoff(targetHeight=tarHeight, duration=3.0)
	timeHelper.sleep(2.0)
	
	
def landCallback():	
	startHoverMatrix = []		
	
	for cf in allcfs.crazyflies:
		startHoverMatrix.append(cf.initialPosition + np.array([0, 0, 0.5]))
	cftoGoals(startHoverMatrix)
		
	allcfs.land(targetHeight=0.005, duration=3.0)
	timeHelper.sleep(3.0)

def cftoGoals(goalPositionMatrix):
	robotPositionMatrix = []
	
	for cf in allcfs.crazyflies:
		robotPositionMatrix.append(cf.position())
		
	print "Current robotPositionMatrix"
	print robotPositionMatrix
	
	costMatrix = distance.cdist(robotPositionMatrix, goalPositionMatrix, 'euclidean')
	
	costMatrix = np.square(costMatrix)
	
	row_ind, col_ind = linear_sum_assignment(costMatrix)
	
	numGoal = len(col_ind)
	
	robotGoalMatrix = np.zeros((numGoal, 3))
	
	for i in range(0, numGoal):
		robotGoalMatrix[i] = goalPositionMatrix[col_ind[i]]
	
	j = 0
	for cf in allcfs.crazyflies: 
		if robotGoalMatrix[j][2] == 0:  #checks if goal position is empty or not
			robotGoalMatrix[j][0] = robotPositionMatrix[j][0]
			robotGoalMatrix[j][1] = robotPositionMatrix[j][1]
			robotGoalMatrix[j][2] = robotPositionMatrix[j][2]
			j = j + 1
		else:
			j = j + 1
			
	duration = distance.cdist(robotPositionMatrix,robotGoalMatrix, 'euclidean')*10 + 1e-6

	for c in range(numGoal):
		print ("cf.goTo({},0,{})").format(robotGoalMatrix[c], duration[c][c])
		allcfs.crazyflies[c].goTo(robotGoalMatrix[c], 0, duration[c][c])
	
	print np.diag(duration)
	timeHelper.sleep((np.max(np.diag(duration)))+ 0.2)

def hoverCallback():
	initpos = []		
	
	for cf in allcfs.crazyflies:
		initpos.append(cf.initialPosition + np.array([0, 0, 1.5]))
		
	cftoGoals(initpos)

def static():
	folder = 'coordfiles'
	
	print "Input .yaml filename for Drone_Coordinates"
	print (os.listdir(("{0}/{1}").format(filepath, folder)))
	fname = raw_input()
	
	with open(('{0}/{1}.yaml').format(folder, fname), 'r') as c:
		doc = yaml.load(c)
		
	pa = PoseArray()
	
	for c in doc['Drone_Coordinates']: 
		p = Pose()
		p.position.x = c[0]
		p.position.y = c[1]
		p.position.z = c[2]
		p.orientation.w = 1
		pa.poses.append(p)
	
	timeHelper.sleep(2.0)
		
	fliecount = 0
	for cf in allcfs.crazyflies:
		fliecount = fliecount + 1
		
	goalPositionMatrix = np.zeros((fliecount, 3))
	
	k = 0
	for p in pa.poses:
		goalPositionMatrix[k][0] = p.position.x
		goalPositionMatrix[k][1] = p.position.y
		goalPositionMatrix[k][2] = p.position.z
		k = k + 1 
		
	print "Initial goalPositionMatrix"
	print goalPositionMatrix
	
	timeHelper.sleep(2.0)
	
	cftoGoals(goalPositionMatrix)

def dynamic():
	folder = 'trajfiles'	
	
	print "Input .csv filename for trajectory path"
	print (os.listdir(("{0}/{1}").format(filepath, folder)))
	trajfile = raw_input()
	
	trajectory.loadcsv(("{0}/{1}.csv").format(folder, trajfile))
	
	timeScale = 1.0
	
	for cf in allcfs.crazyflies:
		cf.uploadTrajectory(0, 0, trajectory)
	
	print "Back to initial? (y/n)"
	goBack = raw_input()
	
	if goBack == 'y':
		pos = []		
		for cf in allcfs.crazyflies:
			pos.append(cf.initialPosition + np.array([0, 0, 1.0]))
		cftoGoals(pos)
	
	allcfs.startTrajectory(0, timescale=timeScale)
	timeHelper.sleep(trajectory.duration * timeScale)
	
	print "Reverse? (y/n)"
	goReverse = raw_input()
	
	if goReverse == 'y':
		allcfs.startTrajectory(0, timescale=timeScale, reverse=True)
		timeHelper.sleep(trajectory.duration * timeScale)
	
def waypoints():
	#make sure the cfs (by id) specified in the waypoint file are active when testing 
	folder = 'wayfiles'
	
	print "Input the .csv filename with the waypoints"
	print (os.listdir(("{0}/{1}").format(filepath, folder)))
	wayfile = raw_input()
	
	#TODO: find way to update the CFAgents based on active cfs instead of editing csv file
	data = np.loadtxt(("{0}/{1}.csv").format(folder, wayfile), skiprows=1, delimiter=',')
	
	data[data[:,0].argsort()]
	
	waypoints = []
	lastAgent = None
	for row in data:
		if lastAgent is None or lastAgent != row[0]:
			lastTime = 0.0
			
		waypoints.append(Waypoint(
			int(row[0]),
			row[1],
			row[2],
			row[3],
			row[4],
			row[4] - lastTime))
		lastTime = row[4]
        lastAgent = int(row[0])
			
	waypoints.sort()
	
	print(waypoints)
	
	'''
	TODO: make the agent input variable so waypoint csv file doesn't need to changed every run
	EITHER takes agents from active cfs or user input for active agents 
	 op1: have to find a way to assign them appropriately in terms of starting positions
		because goTos don't consider paths and collisions 
		maybe find a way to use CFtoGoals script that assigns to nearest starting pos
		- this assumes all CFs start at the same time and are executing in sync
		- it's as if doing a static formation to start then beginning waypoints 
		
	 op2: take in user input to replace the agents from csv file
		loops are getting weird lol 
		making sure the appropriate active cfs are replacing in the same slots 
		
	for now just change csv file I guess :<<<
		
	'''
	
	#execute waypoints	
	
	lastTime = 0.0
	for waypoint in waypoints:
		if waypoint.arrival == 1:
			pos = [waypoint.x, waypoint.y, waypoint.z]
			cf = allcfs.crazyfliesById[waypoint.agent]
			cf.goTo(pos, 0, 5.0)
			
		elif waypoint.arrival > 0:
			timeHelper.sleep(waypoint.arrival - lastTime)
			lastTime = waypoint.arrival
			pos = [waypoint.x, waypoint.y, waypoint.z]
			cf = allcfs.crazyfliesById[waypoint.agent]
			cf.goTo(pos, 0, waypoint.duration)
		
	timeHelper.sleep(np.max(waypoint.arrival))

	
def follow():
	#target Input
	print "Which CF is the target? (CFbyId)"
	targetCFid = int(raw_input())
	targetCF = allcfs.crazyfliesById[targetCFid]
	
	"""
	TODO: Follow demo
	will follow target
	target => waypoints path, trajectory path, or teleop 
	cfs => follow linear or swarm
	linear: cfs will follow the target one at a time in a beeline manner
		(first cf follows each previous location point of the target
		and is followed by the next cfs) 
	swarm: modeled similarly to avoid target by using a maintained distance
		the swarm will crowd the target as closely as possible 
		while maintaining their safe flight distance
		and the whole swarm reacts to the movements of the target
	"""

##MAKE in a separate file
def avoid():
	'''
	TODO: avoid target
	*avoidtarget.py file has better progress
	target => fly by trajectory path or teleop 
	cfs => will adjust smoothly when they are too near the target or each other
		movement of cfs ideally have them fly far enough away to a safe distance
		but once the target is far enough away and it's safe to return to the
		original positions they should go back so that the whole swarm bounces back after interference
	'''
	cfs = allcfs.crazyflies
	
	#target Input
	print "Which CF is the target? (CFbyId)"
	targetCFid = int(raw_input())
	targetCF = allcfs.crazyfliesById[targetCFid]

	#send CFs to different heights
	print "Send crazyflies to random heights? (y/n)"
	randgo = raw_input()
	
	if randgo == 'y':

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
	
		
	#send target to starting point
	toStart = raw_input("Send to start? (y/n)")
	
	if toStart == 'y':
		targetCF.goTo(np.array([2.0, -2.0, 1.0]), 0, 5.0)
		timeHelper.sleep(5.0)
	
	#use trajectory
	goTraj = raw_input("Use a trajectory? (y/n)")
	
	if goTraj == 'y':
		folder = 'trajfiles'	
		
		print "Input .csv filename for trajectory path"
		print (os.listdir(("{0}/{1}").format(filepath, folder)))
		trajfile = raw_input()
		
		trajectory.loadcsv(("{0}/{1}.csv").format(folder, trajfile))
				
		targetCF.uploadTrajectory(0, 0, trajectory)
	
	print("Press button to start avoiding")
	swarm.input.waitUntilButtonPressed()
	
	targetCF.startTrajectory(0, timescale=1.0)
	timeHelper.sleep(trajectory.duration)


if __name__ == "__main__":
	main()
