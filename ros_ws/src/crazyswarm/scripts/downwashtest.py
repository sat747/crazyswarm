#!/usr/bin/env python

"""
Downwash testing

Lets two drones fly directly above one another at varying distances
to test downwash effects
"""

#import different libraries

import numpy as np #python computational library

from pycrazyswarm import * #crazyswarm 
import uav_trajectory #trajectory classes


swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
breathe = 1.0

def main():

	while True:
		print("Start")
		print("[takeoff, land, go]")
		task = raw_input()
		
		if task == 'takeoff':
			takeoffCallback() 
			timeHelper.sleep(breathe)
		elif task == 'land':
			landCallback()
			timeHelper.sleep(breathe)
			break
		elif task == 'go':
			goCallback()
			timeHelper.sleep(breathe)


def takeoffCallback():	
	tarHeight = 0.5
	allcfs.takeoff(targetHeight=tarHeight, duration=3.0)
	timeHelper.sleep(2.0)
	
	
def landCallback():	
	#startHoverMatrix = []		
	
	for cf in allcfs.crazyflies:
		startHoverMatrix = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
		cf.goTo(startHoverMatrix, 0, 5.0)
		
	timeHelper.sleep(5.0)
	
	#for cf in allcfs.crazyflies:
		#startHoverMatrix.append(cf.initialPosition + np.array([0, 0, 0.5]))
	#cftoGoals(startHoverMatrix)
		
	allcfs.land(targetHeight=0.005, duration=3.0)
	timeHelper.sleep(3.0)


def goCallback():
	#send two drones vertically aligned with prompted height difference
	gap = float(raw_input("height difference"))
	print gap
	
	i = 0
	flies = []
	for cf in allcfs.crazyflies:
		flies.append(cf.id)
		i += 1 
		
	print (flies) 

	pos1 = np.array([0, 0, 0.5 + gap])
	pos2 = np.array([0, 0, 0.5])
	
	allcfs.crazyfliesById[(flies[0])].takeoff(targetHeight=0.5, duration=3.0)
	timeHelper.sleep(1.0)
	allcfs.crazyfliesById[(flies[0])].goTo(pos1, 0, 5.0)
	
	print("press button to continue")
	swarm.input.waitUntilButtonPressed()
	
	allcfs.crazyfliesById[(flies[1])].takeoff(targetHeight=0.5, duration=3.0)
	timeHelper.sleep(1.0)
	allcfs.crazyfliesById[(flies[1])].goTo(pos2, 0, 5.0)
	
	timeHelper.sleep(5.0)
		
if __name__ == "__main__":
	main()
