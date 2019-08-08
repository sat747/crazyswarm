#!/usr/bin/env python
'''
Runs emergency stop function for crazyflies in case of programs crashing midflight
'''

from pycrazyswarm import *

if __name__ == "__main__":
	swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    
    allcfs.emergency()
