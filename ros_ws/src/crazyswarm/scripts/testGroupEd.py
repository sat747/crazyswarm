#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

#only supports two groupMasks???

Z = 2

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs


    for cf in allcfs.crazyflies:
	print cf.id 
	if (cf.id > 0) and (cf.id < 6):
	    allcfs.crazyfliesById[cf.id].setGroupMask(2)
	elif (cf.id > 5) and (cf.id < 11):
	    allcfs.crazyfliesById[cf.id].setGroupMask(1)
	elif (cf.id > 10) and (cf.id < 16):
	    allcfs.crazyfliesById[cf.id].setGroupMask(2)
	elif (cf.id > 15) and (cf.id < 21):
	    allcfs.crazyfliesById[cf.id].setGroupMask(1)
	else:
	    allcfs.crazyfliesById[cf.id].setGroupMask(2)
	print cf.groupMask


    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z, groupMask = 2)
    timeHelper.sleep(1.5 + 2*Z)
    allcfs.land(targetHeight=0.01, duration=1.0 + Z, groupMask = 2)
    timeHelper.sleep(1.5 + 2*Z)

    allcfs.takeoff(targetHeight=Z, duration=1.0 + Z, groupMask = 1)
    timeHelper.sleep(1.5 + 2*Z)
    allcfs.land(targetHeight=0.01, duration=1.0 + Z, groupMask = 1)
    timeHelper.sleep(1.5 + 2*Z)



