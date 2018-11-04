#!/usr/bin/env python

from math import sqrt, exp, pi
from scipy.integrate import quad
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
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


    def motionUpdate(self, msg):
        init = 0
        alfa1 = alfa2 = alfa3 = alfa4 = 3.0

        """Compute motion of robot from the last odometry message to the new odometry message"""
        if init == 0:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w
            (t1, t2, theta) = tf.transformations.euler_from_quaternion((0, 0, z, w))
            lastpose = list([x, y, theta])
            init = 1
        if init == 1:
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w
            (t1, t2, theta) = tf.transformations.euler_from_quaternion((0, 0, z, w))
            dx = msg.pose.pose.position.x - lastpose[0]
            dy = msg.pose.pose.position.y - lastpose[1]
            deltatrans = math.sqrt(dx ** 2 + dy ** 2)
            deltarot1 = math.atan2(dy, dx) - lastpose[2]
            deltarot2 = lastpose[2] - theta - deltarot1
            deltarot1prime = deltarot1 - self.sample(alfa1 * deltarot1 + alfa2 * deltatrans)
            deltatransprime = deltatrans - self.sample(alfa3 * deltatrans + alfa4 * (deltarot1 + deltarot2))
            deltarot2prime = deltarot2 - self.sample(alfa1 * deltarot2 + alfa2 * deltatrans)
            poses = PoseArray()
            # particles.header.frame_id = "map"

            # the for loop in the MCL algorithm goes inside the motionupdate method
            for i in range(0, nParticles):
                xprime =lastpose[0] + deltatransprime *math.cos(lastpose[2]+deltarot1prime)
                yprime =lastpose[1] + deltatransprime *math.sin(lastpose[2]+deltarot1prime)
                thetaprime =lastpose[2] + deltarot1prime + deltarot2prime
                self.particle = Particle(self, xprime, yprime, thetaprime)


    def sample(self, num):

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

    def __init__(self, xMin, yMin, xMax, yMax, nparticles):
        rospy.init_node('monteCarlo', anonymous=True)  # Initialize node, set anonymous=true

        self.particleFilter = ParticleFilter()

    #Initialize particle set
	# set number of particles, standard or set
	#initialize node
	#subscribe data we need
	# /scan
	# odometry pose


	#publish to topic for rviz

	#output Xt -> input for next iteration


    self.posePublisher = rospy.Publisher("Poses", Pose)  # pulisher of position+orioentation to topic poses, type Poses
    self.particlesPublisher = rospy.Publisher("PoseArrays", poseArray)  # publisher of particles in poseArray
    rospy.Subscriber("/RosAria/pose", Odometry, particleFilter.motionUpdate)  # subscriber for odometry to be used for motionupdate
    rospy.Subscriber("/scan", LaserScan, particleFilter.weightUpdate)  # subscribe to kinect scan for the sensorupdate
    rospy.spin()

    def sample(self,num):


    def runMCL(self)):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Publish particles to filter in rviz
            rate.sleep()


if __name__ =="__main__":

    # Map boundaries
    xMin = -30
    xMax = 30
    yMin = -30
    yMax = 30


    # Set number of particles
    nParticles = 100

    # Initialize MCL
    monteCarlo = MCL(xMin, yMin, xMax, yMax, nParticles)
    monteCarlo.runMCL()
