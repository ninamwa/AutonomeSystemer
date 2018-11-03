#!/usr/bin/env python

from math import sqrt, exp, pi
from scipy.integrate import quad
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import poseArray

class Particle(object):
	def __init__(self,x,y,theta):

		#if you want, implement id as input and self.id = id
		self.x = x
		self.y = y
		self.theta = theta

class ParticleFilter(object):
	def __init__(self):
		self.particles = []
		self.weights = []
		self.laser_min_angle = 0
		self.laser_max_angle = 0
		self.laser_min_range = 0
		self.laser_max_range = 0


		#Measurement parameters
		self.lambdaShort = 0
		self.zHit = 0
		self.zShort = 0
		self.zMax = 0
		self.zRand = 0
		self.sigmaHit = 0


	def get_pMax(self, zt):
		if zt == self.laser_max_range:
			return 1
		else:
			return 0

	def get_pRand(self, zt):
		if zt >=0 and zt < self.laser_max_range:
			return 1/self.laser_max_range
		else:
			return 0

	def get_pHit(self,zt,zt_star):
		N = (1/sqrt(2*pi*self.sigmaHit**2))*exp(-0.5*((zt-zt_star)**2) / (self.sigmaHit**2))
		n = (quad(N,0,self.laser_max_range))**-1
		if zt >= 0 and zt < self.laser_max_range:
			return n * N
		else:
			return 0



	def get_pShort(self,zt,zt_star):
		n = 1/(1- exp(-self.lambdaShort*zt_star))
		if zt >= 0 and zt < zt_star:
			return n*self.lamdaShort*exp(-self.lamdaShort*zt)
		else:
			return 0


	#measurement model
	def weightUpdate(self,msg):
		self.laser_min_angle = msg.angle_min
		self.laser_max_angle = msg.angle_max
		self.laser_min_range = msg.range_min
		self.laser_max_range = msg.range_max
		self.weights= []

		q = 1
		for particle in self.particles:
			for i in range(0,len(msg.ranges)):
				zt = msg.range[i]	#(Note: values < range_min or > range_max should be discarded)
				if (zt >= self.laser_min_range or zt <= self.laser_max_range):
					#Kalkulere z_star???

					zt_star = sqrt((xt[0] + zt_true[k] * math.cos(xt[2]+angs[k]))**2 + (xt[1] + zt_true[k] * math.sin(xt[2]+angs[k]))**2)

					p = self.zHit * self.get_pHit(self,zt, zt_star) +self.zShort*self.get_pShort(zt, zt_star) + self.zMax* self.get_pMax(zt) +self.zRand*self.get_pRand(zt)
					q = q * p
					if q == 0:
						q = 1e-20 #If q is zero then reassign q a small probability
			self.weights.append(q) ##LITT USIKKER PÅ HVORDAN VI SKAL GJØRE DETTE?



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


if __name__ == "__main__":
	mcl= MCL()
	#initialize MCL
