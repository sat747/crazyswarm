#!/usr/bin/env python
'''

Generates smooth trajectory paths from live input of points
Uses defined functions in matlab to generate a .csv file
That is read by uav_trajectory.py and uploads to CFs

'''

import time
import numpy as np
import array
import matlab.engine

from pycrazyswarm import * #crazyswarm 
import uav_trajectory #trajectory classes


eng = matlab.engine.start_matlab()
filepath = r'/home/trailCrazyswarm/crazyswarm/matlab/trajectorygen/'
eng.addpath(filepath)

def main():
	



if __name__=="__main__":
	main()
