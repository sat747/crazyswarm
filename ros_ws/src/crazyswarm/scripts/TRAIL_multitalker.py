#!/usr/bin/env python

import numpy as np
import os
import yaml
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from pycrazyswarm import *

rospy.init_node('TRAIL_multitalker', anonymous=True)

def formationsData():
    pub = rospy.Publisher('Assignments', PoseArray, queue_size=3, latch=True)

    shapes = [3, 4, 5]
        
    for i in shapes:
	if rospy.has_param('~Static_{}'.format(i)):

            coords = rospy.get_param('~Static_{}'.format(i))

            pa = PoseArray()

            for c in coords:
      	        p = Pose()
	        p.position.x = c[0]
                p.position.y = c[1]
                p.position.z = c[2]
                p.orientation.w = 1
                pa.poses.append(p)

            rospy.sleep(rospy.Duration(7.0))
            pub.publish(pa)

    	else:
            print "Could not find Drone Coordinates"

def dynamicsData():
    pub = rospy.Publisher('Trajectories', String, queue_size=3, latch=True)
    rospy.sleep(rospy.Duration(5.0))
    pub.publish('fig8traj.csv')
    rospy.sleep(rospy.Duration(5.0))
    pub.publish('circletraj.csv')
    rospy.sleep(rospy.Duration(5.0))

def TRAIL_multitalker():
    rospy.sleep(rospy.Duration(2.0))

    formationsData()

    rospy.sleep(rospy.Duration(20.0)) 
    #change based on complexity of formations

    dynamicsData()

if __name__ == '__main__':
    try:
        TRAIL_multitalker()
    except rospy.ROSInterruptException:
        pass
