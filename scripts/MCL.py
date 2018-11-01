#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import poseArray





class Particle(object):
    def __init__(self,x,y,theta):

        #if you want, implement id as input and self.id = id
        self.x = x
        self.y = y
        self.theta = theta



class MCL(object):

    def __init__(self):

    #Initialize particle set
    # set number of particles, standard or set
    #initialize node
    #subscribe data we need
    # /scan
    # odometry pose


    #publish to topic for rviz

    #output Xt -> input for next iteration


    #xtm = sample_motion_model(ut,xt-1m)
    #publish
    #wtm = measurementmodel(zt,xtm,m)

    self. # pulisher of poses type Poses
    self.particlesPublisher = rospy.Publisher("particles", poseArray)
    rospy.Subscriber("/RosAria/pose", Odometry, motionupdate)
    rospy.Subscriber("/scan", LaserScan, sensorupdate)
    rospy.spin()

    def motionupdate(self, ):

    def sensorupdate(self, ):





if __name__ == "__main__":
    main()
    #initialize MCL