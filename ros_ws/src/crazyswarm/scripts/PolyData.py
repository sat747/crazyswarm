#!/usr/bin/env python

#Import necessary libraries
#
import numpy as np
import os
import yaml
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

#Begin PolyData
#
def PolyData():
    #Initialize node
    #
    rospy.init_node('PolyData', anonymous=True)

    #Set Topic, Message Type, Queue Size and Latch Respectively
    #
    pub = rospy.Publisher('Assignments', PoseArray, queue_size=1, latch=True)

    #Check to see if ROS can find the Parameter Drone_Coordinates
    #This is the structure name of the Yaml File
    #Seen in first line of Yaml File
    #If ROSpy finds Drone_Coordinates it saves the contents of the file to the variable coords
    #
    sides = [1, 2, 3, 4, 5]
    for i in sides:
        coords = rospy.get_param('~poly_{}'.format(i))
        pa = PoseArray()

        for c in coords:
      	    p = Pose()
	    p.position.x = c[0]
            p.position.y = c[1]
            p.position.z = c[2]
            p.orientation.w = 1
            pa.poses.append(p)

        rospy.sleep(rospy.Duration(2.0))
        pub.publish(pa)


if __name__ == '__main__':
    try:
        PolyData()
    except rospy.ROSInterruptException:
        pass
