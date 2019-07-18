#!/usr/bin/env python

import yaml
import os
import rospy
from pycrazyswarm import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
breathe = 1.0
filepath = '/home/trailCrazyswarm/crazyswarm/ros_ws/src/crazyswarm/scripts'

if __name__ == "__main__":
	
	folder = 'coordfiles'
	print 'which file'
	
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
	
	i = 0
	flies = []
	for cf in allcfs.crazyflies:
		flies.append(cf.id)
		i += 1 
		
	print (flies) 
	
	allcfs.takeoff(targetHeight=0.7, durationg=3.0)
	
	print("press button to continue")
	swarm.input.waitUntilButtonPressed()
	
	for cf in allcfs.crazyflies:
		cf.goTo(
	
