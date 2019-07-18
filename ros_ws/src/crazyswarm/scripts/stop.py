#!/usr/bin/env python

from pycrazyswarm import *

if __name__ == "__main__":
	swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    allcfs.emergency()
