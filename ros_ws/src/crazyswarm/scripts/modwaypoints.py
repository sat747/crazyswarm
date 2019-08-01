#!/usr/bin/env python

"""
waypoints
	
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

cfcount = 0
for cf in allcfs.crazyflies:
	cfcount = cfcount + 1


def main():
	takeoffCallback()
	
	waypoints()
	
	swarm.input.waitUntilButtonPressed()
	
	landCallback()
	

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
		if robotGoalMatrix[j][2] == 0 and robotGoalMatrix[j][1] == 0 and robotGoalMatrix[j][0] == 0:  #checks if goal position is empty or not
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
	
	for cf in allcfs.crazyflies:
		currpos = np.array(cf.position())
		initpos = np.array(cf.initialPosition)
		if abs(currpos[0] - initpos[0]) <= 0.005 and abs(currpos[1] - initpos[1]) <= 0.005:
			cf.land(targetHeight=0.001, duration=3.0)
			

def hoverCallback():
	initpos = []		
	
	for cf in allcfs.crazyflies:
		initpos.append(cf.initialPosition + np.array([0, 0, 1.5]))
		
	cftoGoals(initpos)


	
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
	
	
	startmatrix = np.zeros((cfcount, 3))
	i = 0
	for waypoint in waypoints:
		if waypoint.arrival == 0:
			startmatrix[i][0] = waypoint.x
			startmatrix[i][1] = waypoint.y
			startmatrix[i][2] = waypoint.z
			i = i + 1
			
	cftoGoals(startmatrix)
	timeHelper.sleep(2.0)
	
	activecfs = []
	for cf in allcfs.crazyflies:
		print cf.id, cf.position()
		pos = np.array(cf.position())
		if pos[2] > 0.005:
			activecfs.append(cf.id)
			
	lastTime = 0.0
	for i in range(activecfs):
		##### doesn't assign to specifically labeled cf/waypoint
		for waypoint in waypoints:
			if waypoint.arrival > 0:
				timeHelper.sleep(waypoint.arrival - lastTime)
				lastTime = waypoint.arrival
				pos = [waypoint.x, waypoint.y, waypoint.z]
				cf = allcfs.crazyfliesById[waypoint.agent]
				cf.goTo(pos, 0, waypoint.duration)
		
	timeHelper.sleep(np.max(waypoint.arrival))



if __name__ == "__main__":
	main()
