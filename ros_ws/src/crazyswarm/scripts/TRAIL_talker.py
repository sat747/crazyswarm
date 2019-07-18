#!/usr/bin/env python

"""
Description of overall project:
This talker node works with the node TRAIL_listener (through the topic Assignments) to send the drones specific locations inside the 3D motion capture system

Node Input:
Yaml file

Description of Node:
**TRAIL_talker Renamed from 'talker'
The TRAIL_talker node currently reads only the parameter given to it (seen in the test_talker launch file in the TRAIL_talker node section). This Yaml file input has the goal positions
of the drones. TRAIL_talker reads the positions into a Pose Array and using the geometry_msgs library from ROS sends the Pose Array to TRAIL_listener through the topic Assignments.

Other important files to look at and their locations: 
#This is the other node we will primarily be using and modifying
home/trail/crazyswarm/ros_ws/src/crazyswarm/scripts/TRAIL_listener.py

#This is the launch file that connects and initiates all of our code
/home/trail/crazyswarm/ros_ws/src/crazyswarm/launch/test_talker.launch

#These two files are two shapes that we have designed. They are good examples for 
#Future designs
/home/trail/crazyswarm/ros_ws/src/crazyswarm/launch/Triangles.yaml
/home/trail/crazyswarm/ros_ws/src/crazyswarm/launch/Coordinates.yaml

#This yaml file has all the starting locations of each of the drones. It must be 
#modified before another drone is added
/home/trail/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml
"""

#Import necessary libraries
#
import numpy as np
import os
import yaml
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

#Begin TRAIL_talker
#
def TRAIL_talker():
    #Initialize node
    #
    rospy.init_node('TRAIL_talker', anonymous=True)

    #Set Topic, Message Type, Queue Size and Latch Respectively
    #
    pub = rospy.Publisher('Assignments', PoseArray, queue_size=1, latch=True)

    #Check to see if ROS can find the Parameter Drone_Coordinates
    #This is the structure name of the Yaml File
    #Seen in first line of Yaml File
    #If ROSpy finds Drone_Coordinates it saves the contents of the file to the variable coords
    #
    if rospy.has_param('~Drone_Coordinates'):
        coords = rospy.get_param('~Drone_Coordinates')
    else:
        #Else it will print could not find Drone_Coordinates
        #
        print "Could not find Drone_Coordinates"

    #Create PoseArray to store new vector in
    #
    pa = PoseArray()

    #Cycle through coordinates in file
    #
    for c in coords:
        #Create a pose (single vector) and add each single vector to the Pose Array
        #
        p = Pose()
        #Store X, Y, and Z position into Pose
        #
        p.position.x = c[0]
        p.position.y = c[1]
        p.position.z = c[2]
        p.orientation.w = 1
        #Append Pose to PoseArray (pa)
        #
        pa.poses.append(p)

    #This Sleep function gives the publisher time to register with the master
    #
    rospy.sleep(rospy.Duration(5.0))

    #Publish the Pose Array to the topic (chatter)
    #
    pub.publish(pa)

if __name__ == '__main__':
    try:
        TRAIL_talker()
    except rospy.ROSInterruptException:
        pass
