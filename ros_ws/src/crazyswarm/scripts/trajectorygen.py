#!/usr/bin/env python
'''

Generates smooth trajectory paths from live input of points
Uses defined functions in matlab to generate polynomial points
That is read by uav_trajectory.py and uploads to CFs

**trajectory points are relative to CF's starting points**
***it worksssss***

'''

import time
import numpy as np
import array
import matlab.engine

from pycrazyswarm import * #crazyswarm 
import uav_trajectory #trajectory classes


swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
trajectory = uav_trajectory.Trajectory()
breathe = 1.0
filepath = '/home/trailCrazyswarm/crazyswarm/ros_ws/src/crazyswarm/scripts'


##Matlab function: pathgen.m##
	#pathgen(name, npts, px, py, pz) 
	#name: .csv file name
	#npts: number of waypoints to be included in trajectory
	#px: array (1d matrix?) of x values (same number as npts) 
	#"" py, pz

def main():
	#ask for inputs: name, npts, x, y, z
	# for avoid target: generate x,y,z's in arrays 
	matlabfilepath = r'/home/trailCrazyswarm/crazyswarm/matlab/trajectorygen/'
	eng.addpath(matlabfilepath)

	
	#print "Input .csv filename to be generated:"
	#csvname = raw_input()
	
	npts = int(raw_input('Number of points:'))
	
	px = []
	py = []
	pz = []
	
	for i in range(npts-1):
		xval = float(input("x val:"))
		px.append(xval)
		yval = float(input("y val:"))
		py.append(yval)
		zval = float(input("z val:"))
		pz.append(zval)
		
	print 'x', px, 'y', py, 'z', pz
	
	#sends inputs through pathgen.m fxn to generate poly points for trajectory 
	duration, ppx, ppy, ppz, ppyaw = eng.pathgen(npts, px, py, pz, nargout=5)
	
	#print duration, ppx, ppy, ppz, ppyaw

	takeoffCallback()
	
	cfFly(duration, ppx, ppy, ppz, ppyaw)
	
	print "Press button to land"
	swarm.input.waitUntilButtonPressed()
	
	allcfs.land(targetHeight=0.005, duration=3.0)
	
	timeHelper.sleep(5.0)
			

def takeoffCallback():
	tarHeight = 0.5
	allcfs.takeoff(targetHeight=tarHeight, duration=3.0)
	timeHelper.sleep(2.0)

def cfFly(duration, ppx, ppy, ppz, ppyaw):
	#folder = 'trajfiles'	
	
	#trajectory.loadcsv(("{0}/{1}.csv").format(folder, csvname))
	
	#sends poly points output from matlab to loadpoly function in uav_trajectory, class Trajectory	
	trajectory.loadpoly(duration, ppx, ppy, ppz, ppyaw)
	
	timeScale = 2.0
	
	for cf in allcfs.crazyflies:
		cf.uploadTrajectory(0, 0, trajectory)
	
	print "Back to initial? (y/n)"
	goBack = raw_input()
	
	if goBack == 'y':
		pos = []		
		for cf in allcfs.crazyflies:
			pos.append(cf.initialPosition + np.array([0, 0, 1.0]))
			allcfs.goTo(cf.initialPosition + np.array([0, 0, 1.0]))

	
	allcfs.startTrajectory(0, timescale=timeScale)
	timeHelper.sleep(trajectory.duration * timeScale)
	
	print "Reverse? (y/n)"
	goReverse = raw_input()
	
	if goReverse == 'y':
		allcfs.startTrajectory(0, timescale=timeScale, reverse=True)
		timeHelper.sleep(trajectory.duration * timeScale)
	



if __name__=="__main__":
	eng = matlab.engine.start_matlab()

	main()
	
	eng.quit()

