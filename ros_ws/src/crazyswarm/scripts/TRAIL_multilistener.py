#!/usr/bin/env python

import rospy
import numpy as np
import time
import array
import uav_trajectory #trajectories
from scipy.spatial import distance
from scipy.optimize import linear_sum_assignment
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from std_msgs.msg import String
from pycrazyswarm import *

rospy.init_node('TRAIL_multilistener', anonymous=True)

swarm = Crazyswarm()

def sendCFtoGoals(goalPositionMatrix):

    allcfs = swarm.allcfs

    robotPositionMatrix = []

    for cf in allcfs.crazyflies:
        robotPositionMatrix.append(cf.position())

    print "robotPositionMatrix"
    print robotPositionMatrix

    #            Beginning of Hungarian Algorithm             #

    costMatrix = distance.cdist(robotPositionMatrix, goalPositionMatrix, 'euclidean')

    costMatrix = np.square(costMatrix)

    row_ind, col_ind = linear_sum_assignment(costMatrix)

    numGoal = len(col_ind)

    robotGoalMatrix = np.zeros((numGoal,3))
    
    for i in range(0, numGoal):
        robotGoalMatrix[i] = goalPositionMatrix[col_ind[i]]
 
    j = 0
    for cf in allcfs.crazyflies: 
	if robotGoalMatrix[j][2] == 0: 
  	    robotGoalMatrix[j][0] = robotPositionMatrix[j][0]
            robotGoalMatrix[j][1] = robotPositionMatrix[j][1]
            robotGoalMatrix[j][2] = robotPositionMatrix[j][2]
	    j = j + 1
 	else:
 	    j = j + 1

    duration = distance.cdist(robotPositionMatrix,robotGoalMatrix, 'euclidean')*10 + 1e-6

    for c in range(numGoal):
        print ("cf.goTo({},0,{})").format(robotGoalMatrix[c], duration[c][c])
        allcfs.crazyflies[c].goTo(robotGoalMatrix[c], 0, duration[c][c])

    print np.max(np.diag(duration))
    print np.diag(duration)
    time.sleep(np.max(np.diag(duration)))
    
#                      rostopic pub /land std_msgs/Empty "{}"
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def landCallback(data):
    allcfs = swarm.allcfs
    goalPositionMatrix = []

    for cf in allcfs.crazyflies:
        goalPositionMatrix.append(cf.initialPosition + np.array([0.0, 0.0, 0.1]))

    sendCFtoGoals(goalPositionMatrix)

    for cf in allcfs.crazyflies:
        cf.land(0.1,10)

def poseCallback(data):
    fliecount = 0
    allcfs = swarm.allcfs 

    for cf in allcfs.crazyflies: 
	fliecount = fliecount + 1

    goalPositionMatrix = np.zeros((fliecount,3))

    i = 0
    for p in data.poses:
        goalPositionMatrix[i][0] = p.position.x
        goalPositionMatrix[i][1] = p.position.y
        goalPositionMatrix[i][2] = p.position.z
        i = i + 1 
    print "goalPositionMatrix"
    print goalPositionMatrix  

    sendCFtoGoals(goalPositionMatrix)

#dynamic trajectories function
def dynamicCallback(data):
    print data

    allcfs = swarm.allcfs
    goalPositionMatrix = []

    #sends bots back to hover over initial positions
    #comment out to retain final formation during trajectories
    for cf in allcfs.crazyflies:
        goalPositionMatrix.append(cf.initialPosition + np.array([0.0, 0.0, 1.5]))
    sendCFtoGoals(goalPositionMatrix)
    
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(data.data)

    for cf in allcfs.crazyflies:
	cf.uploadTrajectory(0, 0, traj)
    
    print traj

    allcfs.startTrajectory(0, timescale=2.0)
    time.sleep(traj.duration + 3.0)
    allcfs.startTrajectory(0, timescale=2.0, reverse=True)
    time.sleep(traj.duration + 3.0)

  
def TRAIL_multilistener():
    for cf in swarm.allcfs.crazyflies:
        print cf.position()
    swarm.allcfs.takeoff(targetHeight=0.5, duration=7.0)
    time.sleep(5.0)

    rospy.Subscriber('Assignments', PoseArray, poseCallback)

    rospy.Subscriber('Trajectories', String, dynamicCallback)

    rospy.Subscriber('land', Empty, landCallback)

    swarm.input.waitUntilButtonPressed()
  

if __name__ == '__main__':
    TRAIL_multilistener()
