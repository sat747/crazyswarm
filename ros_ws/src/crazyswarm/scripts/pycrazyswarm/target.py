#!/usr/bin/env python


import sys
import yaml
import rospy
import numpy as np
import time
import tf_conversions
from std_srvs.srv import Empty
import std_msgs
from tf import TransformListener

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])

class Target:
    def __init__(self, id, initialPosition, tf):
        self.id = id
        prefix = "/tg" + str(id)
        self.prefix = prefix
        self.initialPosition = np.array(initialPosition)

        self.tf = tf

        rospy.wait_for_service(prefix + "/update_params")
        self.updateParamsService = rospy.ServiceProxy(prefix + "/update_params", UpdateParams)

        self.cmdFullStatePublisher = rospy.Publisher(prefix + "/cmd_full_state", FullState, queue_size=1)
        self.cmdFullStateMsg = FullState()
        self.cmdFullStateMsg.header.seq = 0
        self.cmdFullStateMsg.header.frame_id = "/world"

        self.cmdStopPublisher = rospy.Publisher(prefix + "/cmd_stop", std_msgs.msg.Empty, queue_size=1)

    def position(self):
        self.tf.waitForTransform("/world", "/tg" + str(self.id), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/tg" + str(self.id), rospy.Time(0))
        return np.array(position)

    def getParam(self, name):
        return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService([name])

    def setParams(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService(params.keys())

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        self.cmdFullStateMsg.header.stamp = rospy.Time.now()
        self.cmdFullStateMsg.header.seq += 1
        self.cmdFullStateMsg.pose.position.x    = pos[0]
        self.cmdFullStateMsg.pose.position.y    = pos[1]
        self.cmdFullStateMsg.pose.position.z    = pos[2]
        self.cmdFullStateMsg.twist.linear.x     = vel[0]
        self.cmdFullStateMsg.twist.linear.y     = vel[1]
        self.cmdFullStateMsg.twist.linear.z     = vel[2]
        self.cmdFullStateMsg.acc.x              = acc[0]
        self.cmdFullStateMsg.acc.y              = acc[1]
        self.cmdFullStateMsg.acc.z              = acc[2]
        self.cmdFullStateMsg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
        self.cmdFullStateMsg.twist.angular.x    = omega[0]
        self.cmdFullStateMsg.twist.angular.y    = omega[1]
        self.cmdFullStateMsg.twist.angular.z    = omega[2]
        self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)

    def cmdStop(self):
        self.cmdStopPublisher.publish(std_msgs.msg.Empty())

    #
    # wrappers around the parameter setting system for common cases
    #
