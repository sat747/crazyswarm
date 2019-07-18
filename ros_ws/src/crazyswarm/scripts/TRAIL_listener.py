#!/usr/bin/env python

"""
Node Input: PoseArray from Assignments (Renamed Chatter). 

Node Description: 
**TRAIL_listener Renamed from 'listener'
This node takes the data from the Assignments topics and puts it through the Hungarian Algorithm (Also known as the Munkres Algorithm) in order to determine which drone should go to which position based on distance (The lowest position will go to that specific location)

If you want to run a simulation you have to edit test_talker.launch by putting --sim between the quotation marks in args=, so it will look like: args="--sim"
"""

#TODO: 
#  (B): if more goals than drones are assigned (CAPT study: drones will go to nearest goals first then go to a secondary goal, or error message) 
#^ right now, simulation just stops running (which is not the worst but lol)  
#
#TODO: include path processing to minimize downwash effect (maintaining an effective distance between each drone following ellipsoid recommended shape from [1] 
#



#Import necessary Libraries
#
import rospy
import numpy as np
import time
import array
from scipy.spatial import distance
from scipy.optimize import linear_sum_assignment
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from pycrazyswarm import *

#Initialize Node TRAIL_listener, and setting anonymous to be True
#
rospy.init_node('TRAIL_listener', anonymous=True)
#
#Initialize the Crazyswarm API by setting Crazyswarm equal to swarm.
#Now we have the ability to use the functions in the API
#
swarm = Crazyswarm()

#The sendCFtoGoals function 
def sendCFtoGoals(goalPositionMatrix):
    #API command to access the information of each crazyflie
    #
    allcfs = swarm.allcfs
    #Create matrix that contains the coordinates of each robot
    #
    robotPositionMatrix = []

    #For each of the Crazyflies in allcfs append their current position to the robotPositionMatrix
    #The .position function is also from the API.
    #
    for cf in allcfs.crazyflies:
        robotPositionMatrix.append(cf.position())

    #Print the results
    print "robotPositionMatrix"
    print robotPositionMatrix


    #=============================================================================#
    #                                                                             #
    #                     Beginning of Hungarian Algorithm                        #
    #                                                                             #
    #=============================================================================#
    
    #Computes the distance between each pair in the robotPositionMatrix and
    #the goalPositionMatrix using euclidean distance metrics
    #
    costMatrix = distance.cdist(robotPositionMatrix, goalPositionMatrix, 'euclidean')
    #Takes the square root of each element in the costMatrix array.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # **CHECK -- Square or square root? 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    costMatrix = np.square(costMatrix)

    #linear_sum_assignment calculates the minimum, weights between the Costs
    #Its input must be a cost matrix
    #Its output is an array of row indices and column indices
    #We will only need the column indices to send the drones to their desired location
    #
    row_ind, col_ind = linear_sum_assignment(costMatrix)

    
    #Create the desired number of columns that will be used for loop
    #
    numGoal = len(col_ind)

    #Create array of zeroes that we can append the locations to
    #
    robotGoalMatrix = np.zeros((numGoal,3))
    
    #For each i in range 0 to numGoal - 1 input the goalPositionMatrix indice in
    #col_ind[i].
    #EXAMPLE: for robot zero, linear_sum_assignments calculated the lowest cost
    #position for robot zero to fly to and stored the index of that position
    #in space zero in its col_in
    #
    #Now when we call col_ind inside of goalPositionMatrix we get the correct position
    #we should be sending drone zero to in the robotGoalMatrix
    #
    #The output will be an array of positions where each row represents the number
    #robot that should be sent to that particular position
    #
    #==============================================================================#
    #                                                                              #
    #                    robot 0: position robot zero will be sent to              #
    #                    robot 1: position robot one will be sent to               #
    #                    robot 2: position robot two will be sent to               #
    #                                                                              #
    #==============================================================================#
    #
    for i in range(0, numGoal):
        robotGoalMatrix[i] = goalPositionMatrix[col_ind[i]]
        #print(robotGoalMatrix)

    #(A) All extra crazyflies are reassigned to their initial positions if # of flies > # of goal positions 
    #(1) this returns cfs to initial hover positions of the extra flies   
    j = 0
    for cf in allcfs.crazyflies: 
		if robotGoalMatrix[j][2] == 0:  #checks if goal position is empty or not
			robotGoalMatrix[j][0] = robotPositionMatrix[j][0]
			robotGoalMatrix[j][1] = robotPositionMatrix[j][1]
			robotGoalMatrix[j][2] = robotPositionMatrix[j][2]
			j = j + 1
		else:
			j = j + 1

    #This calculates the time is would take for each robot to get to its specific
    #position and multiplies it by ten to make the robots move slower
    #
    duration = distance.cdist(robotPositionMatrix,robotGoalMatrix, 'euclidean')*10 + 1e-6

    #Sends the robots to their respective locations and prints their coordinates
    #
    for c in range(numGoal):
        print ("cf.goTo({},0,{})").format(robotGoalMatrix[c], duration[c][c])
        allcfs.crazyflies[c].goTo(robotGoalMatrix[c], 0, duration[c][c])

    #Sleep function for the length of the time it will take the longest drone to get to
    #its position
    #
    #DO NOT USE ros.sleep()
    #It does not work with the simulations and causes compiling errors
    #Use regular time.sleep as seen below
    #
    time.sleep(np.max(duration[:,0]) + 0.5)

#Enter landCallback function
#This function is only entered if an empty message is entered into the command line
#If you want to land the drones open a new terminal and enter the below message into the command line
#
#                      rostopic pub /land std_msgs/Empty "{}"
#
def landCallback(data):
    #Defines all crazyflies into allcfs
    #
    allcfs = swarm.allcfs
    #Creates empty matrix to store drone position in
    #
    goalPositionMatrix = []
    #Find all positions of drones as seen in sendCFtoGoals function
    #
    for cf in allcfs.crazyflies:
        goalPositionMatrix.append(cf.initialPosition + np.array([0.0, 0.0, 0.1]))

    #Sends landing locations to the sendCFtoGoals function
    #
    sendCFtoGoals(goalPositionMatrix)

    #Tells all crazyflies to land using the .land function from the API
    #
    for cf in allcfs.crazyflies:
        cf.land(0.1,10)

def poseCallback(data):
    
    #Creates fliecount to return number of crazyflies being used
    #initPositionMatrix to hold initial locations for excess crazyflies
    fliecount = 0
    allcfs = swarm.allcfs 

    for cf in allcfs.crazyflies: 
		fliecount = fliecount + 1


    # Define an empty array with preallocated memory for the size of data.poses
    #goalPositionMatrix = np.zeros((11,3))


    #Creates goal position Matrix information received by the Assignments topic
    #The row value is how many drones we are currently using (fliecount).
    #
    goalPositionMatrix = np.zeros((fliecount,3))

        
    #Stores data from PoseArray into goalPositionMatrix
    #
    i = 0

    for p in data.poses:
        goalPositionMatrix[i][0] = p.position.x
        goalPositionMatrix[i][1] = p.position.y
        goalPositionMatrix[i][2] = p.position.z
        i = i + 1 
    print "goalPositionMatrix"
    print goalPositionMatrix  


    #Sends goal locations to sendCFtoGoals
    #
    sendCFtoGoals(goalPositionMatrix)
        
def TRAIL_listener():
    #Takeoff command so that the drones do not have trouble moving to their locations
    #directly from takeoff
    #
    for cf in swarm.allcfs.crazyflies:
        print cf.position()
    swarm.allcfs.takeoff(targetHeight=0.5, duration=7.0)
    time.sleep(0.5)

    #Look for messages from topics Assignments and land, message type, function that
    #is using these subscribers
    #
    rospy.Subscriber('Assignments', PoseArray, poseCallback)
    rospy.Subscriber('land', Empty, landCallback)
    
    #rospy.spin() # DONT USE ROS SLEEPING AND SPINNING SINCE IT INTERFERES WITH THE SIMULATION
    swarm.input.waitUntilButtonPressed()

if __name__ == '__main__':
    TRAIL_listener()
