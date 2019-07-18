#!/usr/bin/env python

import matlab.engine
import time

if __name__ == "__main__":
	
	eng = matlab.engine.start_matlab()
	
	filepath = r'/home/trailCrazyswarm/crazyswarm/matlab/trajectorygen/'
	
	eng.addpath(filepath)
	
	print "First number to add:" 
	var1 = float(raw_input())
	
	print "Second number to add:"
	var2 = float(raw_input())
	
	res = eng.addtest(var1, var2)
	print res
	
	time.sleep(5.0)
	
	eng.quit()

#yay it works#
