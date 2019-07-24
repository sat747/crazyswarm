#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

'''
controls cf flight given input of distance, direction and duration 

Only works on x-y plane cuz angular direction

distance: float in meters
direction: radians relative to +x axis?
duration: seconds

**yaw method of teleop will only work for target cf 
we need the math version of the teleop for other cfs to avoid cuz they can't just turn around everytime they need to move 
## currently only takes into account all positive tan values 
every movement is angled within the quadrant
***need to control basis of angles
'''

###figure out how classes work lol
#ask for agent(cf), distance, direction, duration
#i mean it just executes it right?? 



class Teleop:
	def __init__(self, cfid, distance, direction, duration):
		self.cfid = cfid
		self.distance = distance
		self.direction = direction
		self.duration = duration
	
	def dx(self):
		d = self.distance
		theta = np.radians(self.direction) 
		
		dx = np.sqrt((np.square(d))/(1 + np.square(np.tan(theta))))
		return dx
	
	def dy(self):
		theta = np.radians(self.direction) 
		dx = self.dx()
		
		dy = dx * np.tan(theta)
		return dy
		
		
	
