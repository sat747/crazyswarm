#!/usr/bin/env python
'''

Generates smooth trajectory paths from live input of points
Uses defined functions in matlab to generate a .csv file
That is read by uav_trajectory.py and uploads to CFs

**trajectory points are relative to CF's starting points**

'''

### Error messages about reinstalling Matlab engine?###
  #File "trajectorygen.py", line 15, in <module>
    #import matlab.engine
  #File "/usr/local/lib/python2.7/dist-packages/matlab/engine/__init__.py", line 64, in <module>
    #'MathWorks Technical Support for assistance: %s' % e)
#EnvironmentError: Please reinstall MATLAB Engine for Python or contact MathWorks Technical Support for assistance: /usr/local/MATLAB/R2019a/extern/engines/python/dist/matlab/engine/glnxa64/../../../../../../../bin/glnxa64/libssl.so.1.0.0: undefined symbol: EVP_idea_cbc

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

##TODO: Skip the csv part and go straight from matlab to python 

def main():
	#ask for inputs: name, npts, x, y, z
	# for avoid target: generate x,y,z's in arrays 
	eng = matlab.engine.start_matlab()
	matlabfilepath = r'/home/trailCrazyswarm/crazyswarm/matlab/trajectorygen/'
	eng.addpath(matlabfilepath)
	
	print "Input .csv filename to be generated:"
	csvname = raw_input()
	
	npts = raw_input('Number of points:')
	
	px = []
	py = []
	pz = []
	
	for i in range(npts):
		xval = input("x val:")
		px.append(xval)
		yval = input("y val:")
		py.append(yval)
		zval = input("z val:")
		pz.append(zval)
	
	gen = eng.pathgen(csvname, npts, xval, yval, zval)
	
	takeoffCallback()
	
	cfFly(csvname)
	
	print "Press button to land"
	swarm.input.waitUntilButtonPressed()
	
	allcfs.land(targetHeight=0.005, duration=3.0)
	
	eng.quit()
		

def takeoffCallback():
	tarHeight = 0.5
	allcfs.takeoff(targetHeight=tarHeight, duration=3.0)
	timeHelper.sleep(2.0)

def cfFly(csvname):
	folder = 'trajfiles'	
	
	trajectory.loadcsv(("{0}/{1}.csv").format(folder, csvname))
	
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
	



if __name__=="__main__":
	main()
