#!/usr/bin/env python
from math import *
import scipy.integrate as integrate
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf # A little unsure about this one
import numpy
import random

class Particle(object):
	def __init__(self,x,y,theta, weight):

		# METRIC
		self.x = x
		self.y = y
		self.theta = theta
		self.weight = weight

class Map(object):
	def __init__(self,msg):
		self.data = msg.data
		self.width = msg.info.width
		self.height = msg.info.height
		self.resolution = msg.info.resolution #Meter per cell

		# The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
		self.origin = Pose()
		self.origin.position.x = msg.info.origin.position.x
		self.origin.position.y = msg.info.origin.position.y

		# Map boundaries real world METRIC
		self.xMin =  0#-(self.height*self.resolution)/2.0
		self.xMax = self.width*self.resolution
		self.yMin = 0#-(self.width*self.resolution)/2.0
		self.yMax = self.height*self.resolution #(self.width*self.resolution)/2.0


class ParticleFilter(object):
	def __init__(self):
		self.particles = []

		# Set number of particles
		self.nParticles = 10

		self.newParticles = []
		self.laser_min_angle = 0
		self.laser_max_angle = 0
		self.laser_min_range = 0
		self.laser_max_range = 0

		#OccupancyGrid Map
		self.map= None

		self.init_weight = 1/self.nParticles


		# Last odometry measurements
		self.lastOdom = Odometry()

		# Current odometry measurements
		self.Odom = Odometry()

		#Initialize alpha
		self.alfa = [3.0, 3.0, 3.0, 3.0]


		# Relative change in x,y,theta over time dt since the last time particles were updated
		self.dx = 0
		self.dy = 0
		self.dtheta = 0

		# Initialize control signal u from odometry
		self.u = [0, 0, 0]

		#Measurement parameters
		self.lambdaShort = 0.1
		self.zHit = 0.25
		self.zShort = 0.25
		self.zMax = 0.25
		self.zRand = 0.25
		self.sigmaHit = 0.2

	def createMap(self,msg):
		self.map = Map(msg)
		print('New map is made')

	def initializeParticles(self): #metric
		#Initialize particles and distributes uniformly randomly within the workspace.
		for i in range(0,self.nParticles):
			free = True
			while free:
				xi = numpy.random.uniform(self.map.xMin, self.map.xMax)
				yi = numpy.random.uniform(self.map.yMin, self.map.yMax)
				row, col = self.metricToGrid(xi, yi)

				#If the cell is free, there is a probability that the robot is located here
				#used to be (row,col)
				if not(self.isOccupied((row,col))):
					thetai = numpy.random.uniform(0, 2*pi)
					particlei = Particle(xi, yi, thetai, self.init_weight)
					self.particles.append(particlei)
					free = False
		print('All particles initialized')



	def getOdom(self,msg):
		self.lastOdom = self.Odom
		self.Odom = msg

		oldx = self.lastOdom.pose.pose.position.x
		oldy = self.lastOdom.pose.pose.position.y
		oldz = self.lastOdom.pose.pose.orientation.z
		oldw = self.lastOdom.pose.pose.orientation.w
		(t0,t1,oldtheta) = tf.transformations.euler_from_quaternion((0,0,oldz,oldw))
		oldpose = list([oldx,oldy,oldtheta])

		x = self.Odom.pose.pose.position.x
		y = self.Odom.pose.pose.position.y
		z = self.Odom.pose.pose.orientation.z
		w = self.Odom.pose.pose.orientation.w
		(t0, t1, theta) = tf.transformations.euler_from_quaternion((0, 0, z, w))
		newpose = list([x,y,theta])

		#these need to be initialized to zero right after calling predict pose
		self.dx = newpose[0]-oldpose[0]
		self.dy = newpose[1]-oldpose[1]
		self.dtheta = newpose[2]-oldpose[2]

		deltatrans = sqrt(self.dx ** 2 + self.dy ** 2)
		deltarot1 = atan2(self.dy, self.dx) - oldpose[2]
		deltarot2 = oldpose[2] - theta - deltarot1

		self.u = [deltatrans, deltarot1, deltarot2]


git 
	# This needs to be called inside sensorupdate with a for loop
	def predictParticlePose(self,particle):
	#Predict new pose for a particle after action u is performed over a timeinterval dt
		dtbt = self.u[0] - self.sample(self.alfa[2] * self.u[0] + self.alfa[3] * (self.u[1] * self.u[2]))
		dtb1 = self.u[1] - self.sample(self.alfa[0] * self.u[1] + self.alfa[1] * self.u[0])
		dtb2 = self.u[2] - self.sample(self.alfa[0] * self.u[2] + self.alfa[1] * self.u[0])
		newx = particle.x + dtbt * cos(particle.theta + dtb1)
		newy= particle.y + dtbt * sin(particle.theta + dtb1)

		start = self.metricToGrid(particle.x,particle.y)
		current = start
		end = self.metricToGrid(newx,newy)

		cells = self.bresenhamLineAlg(start[0], start[1], end[0], end[1])
		#cells = list(bresenham(start[0], start[1], end[0], end[1]))
		count =0
		#while count < len(cells):
		for grid in cells:
			if(not self.isOccupied(grid)):
				current = grid
			if(self.isOccupied(grid) or (current[0]==end[0] and current[1]==end[1])):
				break
			count +=1

		particle.x = current[0]*self.map.resolution
		particle.y = current[1]*self.map.resolution
		particle.theta = particle.theta + dtb1 + dtb2


	#Creates message of type Pose from Particle()
	# Use when we publish Particles to ROS topic Poses
	def createPose(self, particle):
			msg = Pose()
			msg.position.x = particle.x
			msg.position.y = particle.y
			quaternion = tf.transformations.quaternion_from_euler(0, 0, particle.theta)
			msg.orientation.z = quaternion[2]
			msg.orientation.w = quaternion[3]
			return msg


	#sample from normal distribution
	def sample(self, num):
		su =0
		a =-num
		b= num
		if a >= b:
			a=num
			b=-num
		for i in range(0,12):
			su += numpy.random.random_integers(a,b)
		return 0.5*su


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
		#N = (1/sqrt(2*pi*self.sigmaHit**2))*exp(-0.5*((zt-zt_star)**2) / (self.sigmaHit**2))

		#print('zt:')
		#print(zt)
		#print('zt_true')
		#print(zt_star)
		N1 = 1/sqrt(2*pi*self.sigmaHit**2)
		#print('N2:')
		N2 = exp((-0.5*(zt-zt_star)**2)/(self.sigmaHit**2))
		#print(N2)
		def integrand(x):
			return (1 / sqrt(2 * pi * self.sigmaHit ** 2)) * exp(-0.5 * ((x - zt_star) ** 2) / (self.sigmaHit ** 2))

		n_temp = integrate.quad(integrand,0,self.laser_max_range)
		#print('n_temp:')
		#print(n_temp[0])
		if zt >= 0 and zt < self.laser_max_range:
			return (N1*N2)/n_temp[0]
		else:
			return 0




	def get_pShort(self,zt,zt_star):
		#n = 1/(1-exp(-self.lambdaShort*zt_star))
		if zt >= 0 and zt < zt_star:
			return self.lambdaShort*exp(-self.lambdaShort*zt)
		else:
			return 0

	#Measurement model
	def weightUpdate(self,msg):

		for particle in self.particles:
			self.predictParticlePose(particle)

		self.dx = 0
		self.dy = 0
		self.dtheta = 0

		q = 1
		for particle in self.particles:
			for i in range(0,len(msg.ranges)):
				zt = msg.ranges[i]	#(Note: values < range_min or > range_max should be discarded)
				angle = radians(i) - self.laser_min_angle
				if (zt >= self.laser_min_range or zt <= self.laser_max_range):
					zt_star = self.raycasting(particle,angle)
					p = self.get_pHit(zt,zt_star)
					#p = self.zHit * self.get_pHit(zt,zt_star) +self.zShort*self.get_pShort(zt, zt_star) + self.zMax* self.get_pMax(zt) +self.zRand*self.get_pRand(zt)
					#p = self.zMax * self.get_pMax(zt) + self.zRand * self.get_pRand(zt)
					q = q * p
					if q == 0:
						q = 1e-300 #If q is zero then reassign q a small probability
			particle.weight = q

		#REMOVED FOR DEBUG
		self.newParticles = []
		self.resample()
		self.particles = self.newParticles


	def raycasting(self, particle,angle):
		theta= particle.theta+angle-(pi/2)
		x0,y0 = self.metricToGrid(particle.x,particle.y) #Converting into grid
		x1,y1 = self.metricToGrid(particle.x+self.laser_max_range*cos(theta),particle.y+self.laser_max_range*sin(theta)) #Converting into grid
		#grids = list(bresenham(x0,y0,x1,y1))
		grids=self.bresenhamLineAlg(x0,x1,y0,y1) #Finding all nearby grids to beam line
		# For all nearby grid, check if they are occupied
		for p in grids:
			if self.isOccupied(p):
				return sqrt((p[0]-x0)**2 + (p[1]-y0)**2)*self.map.resolution
		# If none are occupied, return max range
		return self.laser_max_range


	def bresenhamLineAlg(self, x0, y0, x1, y1):
		is_steep = abs(y1 - y0) > abs(x1 - x0)

		# Rotating
		if is_steep:
			x0, y0 = y0, x0
			x1, y1 = y1, x1

		# Swapping start and end
		swapped = False
		if x0 > x1:
			x0, x1 = x1, x0
			y0, y1 = y1, y0
			swapped = True

		dx = x1 - x0
		dy = y1 - y0

		if (dy == 0):
			ystep = 0
		elif (dy < 0):
			ystep = -1
		else:
			ystep = 1

		error = int(dx / 2)
		y = y0
		points = []

		for x in range(x0, x1 + 1):
			if is_steep:
				#was (y,x)
				points.append((y, x))
			else:
				points.append((x, y))

			error -= abs(dy)
			if error < 0:
				y += ystep
				error += dx

		# Reverse the list if swapped
		if swapped:
			points.reverse()
		return(points)

	def metricToGrid(self,x,y):
		# Origin is the real-world pose of the cell (0,0) in the map.
		grid_x = int(x/self.map.resolution) #-self.map.origin.position.x
		grid_y = int(y/self.map.resolution) #-self.map.origin.position.y
		#check if valid grid coordinates
		if (grid_x <0):
		    grid_x=0
		elif (grid_x > self.map.width):
			grid_x=self.map.width
		if (grid_y<0):
			grid_y=0
		elif (grid_y > self.map.height):
			grid_y=self.map.height
		return (grid_x,grid_y)

	#Data is row-major indexed A[0][1] = a12
	def isOccupied(self,grid):
		index = self.findMapIndex(grid)
		if (self.map.data[index-1] == 0):
			return False
		return True


	def findMapIndex(self,grid): #grid(x,y) = row,col
		 #grid[0] = x = row
		 #grid[1] = y = col
		return int(((grid[1]-1)*self.map.width) + grid[0])


	# Resampling the particles to get a new probability distribution Xt. The particles with
	# high weight have a higher probability of being resampled, than the ones with lower.
	def resample(self):
		weights_temp=[]
		s=0
		for particle in self.particles:
			s += particle.weight**2
			weights_temp.append(particle.weight)
		s = sqrt(s)
		#DEBUG
		if(s==0):
			s=0.00001
		weights_temp[:] = [x / s for x in weights_temp]


		cumsum = []
		sum = 0
		for weight in weights_temp:
			sum += weight
			cumsum.append(sum)

		for i in range (0,len(cumsum)):
			rand = numpy.random.uniform(0, 1) * max(cumsum)
			k = 0
			for j in cumsum:
				if rand > j:
					k += 1
			resp = self.particles[k]  # Denne partikkelen skal resamples
			resp.weight = self.init_weight
			self.newParticles.append(resp)


class MCL(object):
	def __init__(self):
		rospy.init_node('monteCarlo', anonymous=True)  # Initialize node, set anonymous=true

		self.particleFilter = ParticleFilter()

		rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
		#To make sure the map is built before the initialization starts:
		rospy.sleep(1)

		# set number of particles, standard or set
		# shall we have no parameters in?

		# Initialize particle set in particle filter
		self.particleFilter.initializeParticles()

		# Check if first publish
		self.it = 0


		# Do something about the time

		self.posePublisher = rospy.Publisher("Poses", Pose, queue_size=10)  # pulish of position+orioentation to topic poses, type Poses
		self.particlesPublisher = rospy.Publisher("PoseArrays", PoseArray, queue_size=10)  # publisher of particles in poseArray
		rospy.Subscriber("/RosAria/pose", Odometry, self.odomCallback)  # subscriber for odometry to be used for motionupdate
		rospy.Subscriber("/scan", LaserScan, self.sensorCallback)  # subscribe to kinect scan for the sensorupdate
		#rospy.spin()

	# for at vi skal kunne "lagre" og ha tilgjengelig tidligere meldinger, maa dette haandteres i Particle filter der
	# ting foregaar.
	# callback trengs for aa kunne lose tidsproblemet (mutex) dette har vi ikke sett paa naa, men
	# implementerer korrekt naa, for aa slippe arbeid senere

	def odomCallback(self, msg):
		self.particleFilter.getOdom(msg)
		#Here we will need something to adjust the time : mutex acquire and release

	def publishPoseArray(self):
		pa = PoseArray()
		pa.header.frame_id = "map"
		pa.header.stamp = rospy.Time.now()
		rate = rospy.Rate(10)  #
		if (self.it==1):
			for particle in self.particleFilter.newParticles:
				msg = self.particleFilter.createPose(particle)
				pa.poses.append(msg)
				rospy.sleep(0.01)
				self.posePublisher.publish(msg)
		if (self.it==0):
			for particle in self.particleFilter.particles:
				msg = self.particleFilter.createPose(particle)
				pa.poses.append(msg)
				rospy.sleep(0.01)
				self.posePublisher.publish(msg)

		rospy.sleep(0.01)
		self.particlesPublisher.publish(pa)
		#self.it =1
		rate.sleep()

			# FORSLAG, men her maa noe gores med tid

	def sensorCallback(self, msg):
		#Laser min/max angle and range are constant and will only be set the first time
		if (self.particleFilter.laser_max_range == 0):
			self.particleFilter.laser_min_angle = msg.angle_min
			self.particleFilter.laser_max_angle = msg.angle_max
			self.particleFilter.laser_min_range = msg.range_min
			self.particleFilter.laser_max_range = msg.range_max
		self.particleFilter.weightUpdate(msg)


	def mapCallback(self, msg):
		self.particleFilter.createMap(msg)

	def runmcl(self):
		rate = rospy.Rate(0.2)
		while not rospy.is_shutdown():
			self.publishPoseArray()
			rate.sleep()


if __name__ =="__main__":

	# Initialize MCL
	# Inputs are needed for Particle Filter, or should we just set them there? NINA
	monteCarlo = MCL()
	monteCarlo.runmcl()
