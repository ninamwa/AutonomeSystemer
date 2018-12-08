#!/usr/bin/env python

from pprint import pprint as pp

from math import *
import scipy.integrate as integrate
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf  # A little unsure about this one
import numpy
from numpy import median
import matplotlib.pyplot as plt
import time
import random


class Particle(object):
    def __init__(self, x, y, theta, weight):
        # METRIC
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class Map(object):
    def __init__(self, msg):
        self.data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution  # Meter per cell

        # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
        self.origin = Pose()
        self.origin.position.x = msg.info.origin.position.x
        self.origin.position.y = msg.info.origin.position.y

        # Map boundaries real world METRIC
        self.xMin = 0
        self.xMax = self.width * self.resolution
        self.yMin = 0
        self.yMax = self.height * self.resolution

class ParticleFilter(object):
    def __init__(self):
        self.particles = []

        # Set number of particles
        self.nParticles = 20

        self.newParticles = []
        self.laser_min_angle = 0
        self.laser_max_angle = 0
        self.laser_min_range = 0
        self.laser_max_range = 0

        # OccupancyGrid Map
        self.map = None

        self.init_weight = 1 / self.nParticles

        # Last odometry measurements
        self.lastOdom = Odometry()

        # Current odometry measurements
        self.Odom = Odometry()

        # Initialize alpha
        self.alfa = [0.01, 0.01, 0.01, 0.01]

        # Relative change in x,y,theta over time dt since the last time particles were updated
        self.dx = 0
        self.dy = 0
        self.dtheta = 0

        self.publ = True
        # Initialize control signal u from odometry
        self.u = [0, 0, 0]

        # Initialize LaserScan Message
        self.scanMsg = LaserScan()

        # Measurement parameters
        self.lambdaShort = 0.1
        self.zHit = 0.4
        self.zShort = 0
        self.zMax = 0.30
        self.zRand = 0.30
        self.sigmaHit = 1
        # Only re-sample when neff drops below a given threshold (M/2)
        self.n_tres = self.nParticles*0.9

        #Augmented MCL parameters
        self.w_slow=1
        self.w_fast=1
        self.w_avg=0
        self.alpha_slow = 0.05
        self.alpha_fast = 0.5

        #Wait for map
        self.wait=False

        # First odometry?
        self.first = True

    def createMap(self, msg):
        self.map = Map(msg)
        print('New map is made')
        self.wait=True

    def initializeParticles(self):  # metric
        # Initialize particles and distributes uniformly randomly within the workspace.
        for i in range(0, self.nParticles):
            free = True
            while free:
                #smaller initialize area, closer to robot
                # For rosbag Nina2, the robot starts here:
                x = 10.47
                y = 9.53
                theta = 0
                #xi = numpy.random.uniform(self.map.xMax/2.8, self.map.xMax/1.7)
                #yi = numpy.random.uniform(self.map.yMax/2.7, self.map.yMax/1.7)
                #xi = numpy.random.uniform(self.map.xMax/4, self.map.xMax/2)
                #yi = numpy.random.uniform(self.map.yMax/4, self.map.yMax/2)
                #xi = numpy.random.uniform(x-1.5, x+1.5)
                #yi = numpy.random.uniform(y-1, y+0.2)

                xi = numpy.random.uniform(x-2,x+2)#self.map.xMin, self.map.xMax)
                yi = numpy.random.uniform(y-2,y)#self.map.yMin, self.map.yMax)


                row, col = self.metricToGrid(xi, yi)

                # If the cell is free, there is a probability that the robot is located here
                # used to be (row,col)
                if not (self.isOccupied((row, col))):
                    thetai = numpy.random.uniform(-pi/6, pi/6)
                    particlei = Particle(xi, yi, thetai, self.init_weight)
                    #particlei = Particle(xi, yi, theta, self.init_weight)
                    self.particles.append(particlei)
                    free = False
        print('All particles initialized')


    def randomParticle(self):
        free = True
        while free:
            xi = numpy.random.uniform(self.map.xMin, self.map.xMax)
            yi = numpy.random.uniform(self.map.yMin, self.map.yMax)
            row, col = self.metricToGrid(xi, yi)
            if not (self.isOccupied((row, col))):
                thetai = numpy.random.uniform(0, 2 * pi)
                free = False
                return Particle(xi, yi, thetai, self.init_weight)


    # sample from normal distribution
    def sample(self, num):
        su = 0
        a = -num
        b = num
        if a >= b:
            a = num
            b = -num
        for i in range(0, 12):
            su += numpy.random.uniform(a, b)
        return 0.5 * su

    def sample2(self, b):
        sum=0
        for i in range(1, 12):
            sum += numpy.random.uniform(-1,1)
        return (b/6)*sum

    def getOdom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        (t0, t1, theta) = tf.transformations.euler_from_quaternion((0, 0, z, w))
        self.Odom = list([x, y, theta])
        return list([x, y, theta])


    def getU(self):

        if self.first:
            msg = Odometry()
            self.lastOdom = self.Odom
            self.first = False

        # these need to be initialized to zero right after calling predict pose
        self.dx = self.Odom[0] - self.lastOdom[0]
        self.dy = self.Odom[1] - self.lastOdom[1]
        self.dtheta = self.Odom[2] - self.lastOdom[2]

        deltatrans = sqrt(self.dx ** 2 + self.dy ** 2)
        deltarot1 = atan2(self.dy, self.dx) - self.lastOdom[2]
        deltarot2 = self.dtheta - deltarot1

        self.u = [deltatrans, deltarot1, deltarot2]

    def predictParticlePose(self, particle):
        # Predict new pose for a particle after action u is performed over a timeinterval dt
        dtbt = self.u[0] - self.sample2(self.alfa[2] * self.u[0] + self.alfa[3] * (self.u[1] + self.u[2]))
        dtb1 = self.u[1] - self.sample2(self.alfa[0] * self.u[1] + self.alfa[1] * self.u[0])
        dtb2 = self.u[2] - self.sample2(self.alfa[0] * self.u[2] + self.alfa[1] * self.u[0])
        newx = particle.x + dtbt * cos(particle.theta + dtb1)
        newy = particle.y + dtbt * sin(particle.theta + dtb1)


        start = self.metricToGrid(particle.x, particle.y)
        end = self.metricToGrid(newx, newy)

        #cells = self.bresenhamLineAlg(start[0], start[1], end[0], end[1])
        count=0
        current = None
        #for p in cells:
        #    if self.isOccupied(p):
        #        current = cells[count-1]
        #        break
        #    count +=1

        if (current is None):
            particle.x = newx
            particle.y = newy
            particle.theta = particle.theta + dtb1 + dtb2
        else:
            particle.x = current[0] * self.map.resolution
            particle.y = current[1] * self.map.resolution
            particle.theta = particle.theta + dtb1 + dtb2


    # Creates message of type Pose from Particle()
    # Use when we publish Particles to ROS topic Poses
    def createPose(self, particle):
        msg = Pose()
        msg.position.x = particle.x
        msg.position.y = particle.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, particle.theta)
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        return msg

    def get_pMax(self, zt):
        if zt == self.laser_max_range:
            return 1
        else:
            return 0

    def get_pRand(self, zt):
        if zt >= 0 and zt < self.laser_max_range:
            return 1 / self.laser_max_range
        else:
            return 0

    def get_pHit(self, zt, zt_star):
        N1 = 1 / sqrt(2 * pi * self.sigmaHit ** 2)
        N2 = exp((-0.5 * (zt - zt_star) ** 2) / (self.sigmaHit ** 2))
        def integrand(x):
            return (1 / sqrt(2 * pi * self.sigmaHit ** 2)) * exp(-0.5 * ((x - zt_star) ** 2) / (self.sigmaHit ** 2))

        n_temp = integrate.quad(integrand, 0, self.laser_max_range)
        if zt >= 0 and zt < self.laser_max_range:
            return (N1 * N2) / n_temp[0]
        else:
            return 0

    def get_pShort(self, zt, zt_star):
        # n = 1/(1-exp(-self.lambdaShort*zt_star))
        if zt >= 0 and zt < zt_star:
            return self.lambdaShort * exp(-self.lambdaShort * zt)
        else:
            return 0

    def minDistance(self,x,y):
        bol=True
        row1,col1 = self.metricToGrid(x,y)
        queue = [(row1,col1)]
        checked=[]
        while bol:
            grid=queue[0]
            if grid in checked:
                queue.remove(grid)
            else:
                if (grid[0] > 0):
                    queue.append((grid[0] - 1, grid[1]))
                if (grid[0]< self.map.xMax):
                    queue.append((grid[0] + 1, grid[1]))
                if(grid[1]<self.map.yMax):
                    queue.append((grid[0], grid[1] + 1))
                if (grid[1]>0):
                    queue.append((grid[0], grid[1] - 1))
                if self.isOccupied(grid):
                    bol=False
                    dist = sqrt((grid[0] - row1) ** 2 + (grid[1] - col1) ** 2) * self.map.resolution
                    return dist**2
                queue.remove(grid)
                checked.append(grid)

    def prob(self,a,b):
        return (exp(-0.5*(a**2/b)/sqrt(2*pi*b)))

    def likelihood_field_range(self,particle,msg):
        q=1
        c=0
        row, col = self.metricToGrid(particle.x, particle.y)
        if (self.isOccupied((row, col))):
            q=0

        else:
            for i in range(0, len(msg.ranges)):
                if(c%15==0):
                    ztk=msg.ranges[i]
                    angle = radians(i)
                    theta = particle.theta + angle - (pi / 2)
                    x_sens = particle.x + self.laser_max_range * cos(theta)
                    y_sens = particle.y + self.laser_max_range * sin(theta)
                    if(ztk != numpy.inf):
                        if (ztk >= self.laser_min_range or ztk <= self.laser_max_range):
                            x= particle.x +x_sens*cos(particle.theta) - y_sens*sin(particle.theta) + ztk*cos(particle.theta + theta)
                            y= particle.y + y_sens*sin(particle.theta)+ x_sens*sin(particle.theta) + ztk*sin(particle.theta + theta)

                            dist = self.minDistance(x,y)
                            q = q*(self.zHit*self.prob(dist,self.sigmaHit**2)+(self.zRand/self.zMax))
                c+=1
        return q

    def beam_range_finder(self,particle,msg):
        q=1
        c=0
        for i in range(0, len(msg.ranges)):
            if (c%20==0):
                zt = msg.ranges[i]  # (Note: values < range_min or > range_max should be discarded)
                angle = radians(i) - self.laser_min_angle
                if (zt >= self.laser_min_range or zt <= self.laser_max_range):
                    zt_star = self.raycasting(particle, angle)
                    p = self.get_pHit(zt, zt_star)
                    # p = self.zHit * self.get_pHit(zt,zt_star) +self.zShort*self.get_pShort(zt, zt_star) + self.zMax* self.get_pMax(zt) +self.zRand*self.get_pRand(zt)
                    # p = self.zMax * self.get_pMax(zt) + self.zRand * self.get_pRand(zt)
                    q = q * p
                    if q == 0:
                        q = 1e-300  # If q is zero then reassign q a small probability
            c+=1
        return q


    # Measurement model
    def weightUpdate(self, msg):
        q = 1
        self.w_avg=0
        for particle in self.particles:

            particle.weight = self.likelihood_field_range(particle,msg)
            #particle.weight = self.beam_range_finder(particle,msg)

        self.w_avg = self.w_avg+((1.0/self.nParticles)*particle.weight)

        # Normalize weights to find n_eff
        weights_temp = []
        s= 0.0
        for particle in self.particles:
            s += particle.weight
            weights_temp.append(particle.weight)
        if(s==0):
            s=0.001
        weights_temp = [x / s for x in weights_temp]

        temp = 0

        for weight in weights_temp:
            temp = temp + weight**2

        if (temp==0):
            temp=0.001

        n_eff = 1.0 / temp



        #Only re-sample when n_eff dnops below a given threshold n_tres
        if n_eff < self.n_tres:
            #hh = self.low_variance_sampler()
            self.resample()
            print("I just resampled")
        if n_eff >= self.n_tres:
            self.newParticles = self.particles
            print("I did not resample")
        self.particles = self.newParticles

    def raycasting(self, particle, angle):
        theta = particle.theta + angle - (pi / 2)
        x0, y0 = self.metricToGrid(particle.x, particle.y)  # Converting into grid
        x1, y1 = self.metricToGrid(particle.x + self.laser_max_range * cos(theta),particle.y + self.laser_max_range * sin(theta))  # Converting into grid
        # grids = list(bresenham(x0,y0,x1,y1))
        grids = self.bresenhamLineAlg(x0, y0, x1, y1)  # Finding all nearby grids to beam line
        # For all nearby grid, check if they are occupied
        for p in grids:
            if self.isOccupied(p):
                return sqrt((p[0] - x0) ** 2 + (p[1] - y0) ** 2) * self.map.resolution
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
                # was (y,x)
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
        return (points)

    def metricToGrid(self, x, y):
        # Origin is the real-world pose of the cell (0,0) in the map.
        grid_x = int(x / self.map.resolution)  # -self.map.origin.position.x
        grid_y = int(y / self.map.resolution)  # -self.map.origin.position.y
        # check if valid grid coordinates
        if (grid_x < 0):
            grid_x = 0
        elif (grid_x > self.map.width):
            grid_x = self.map.width
        if (grid_y < 0):
            grid_y = 0
        elif (grid_y > self.map.height):
            grid_y = self.map.height
        return (grid_x, grid_y)

    # Data is row-major indexed A[0][1] = a12
    def isOccupied(self, grid):
        index = self.findMapIndex(grid)
        if (self.map.data[index - 1] == 0):
            return False
        return True

    def findMapIndex(self, grid):
        return int(((grid[1] - 1) * self.map.width) + grid[0])


    # Resampling the particles to get a new probability distribution Xt. The particles with
    # high weight have a higher probability of being resampled, than the ones with lower.
    def resample(self):

        self.w_slow = self.w_slow + self.alpha_slow*(self.w_avg-self.w_slow)
        self.w_fast = self.w_fast + self.alpha_fast*(self.w_avg-self.w_fast)
        w = max(0,1-(self.w_fast/self.w_slow))
        #w=0
        weights_temp = []
        s = 0
        for particle in self.particles:
            weights_temp.append(particle.weight)
        weights_temp = numpy.array(weights_temp)

        weights_temp=weights_temp/weights_temp.sum()
        bol=False
        if (w != 0):
            x = numpy.insert(weights_temp,len(weights_temp),w)
            x = x/x.sum()
            weights_temp=x
            bol=True


        cumsum = []
        sum = 0
        for weight in weights_temp:
            sum += weight
            cumsum.append(sum)
        self.newParticles = []
        while (len(self.newParticles) != self.nParticles):
            rand = numpy.random.uniform(0, 1) * max(cumsum)
            k = 0
            for j in cumsum:
                if rand > j:
                    k += 1
            if (k == len(weights_temp)-1 and bol):
                self.newParticles.append(self.randomParticle())
            else:
                rp = self.particles[k]  # resample_particle
                resp = Particle(rp.x, rp.y, rp.theta, self.init_weight)  # Denne partikkelen skal resamples
                self.newParticles.append(resp)
        bol=False


class MCL(object):
    def __init__(self):
        rospy.init_node('monteCarlo', anonymous=True)  # Initialize node, set anonymous=true

        self.particleFilter = ParticleFilter()

        rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
        # To make sure the map is built before the initialization starts:
        while not self.particleFilter.wait:
            if (self.particleFilter.wait):
                break

        # Check if we have new messages
        self.newodom = False
        self.newscan = False
        self.odomstamp = time.time()
        self.endx=0
        self.endy=0

        # Initialize particle set in particle filter
        self.particleFilter.initializeParticles()

        # Error amcl vs marker
        self.timelist = []
        self.errorlist = []

        # Initialize pos message
        self.pos = PoseWithCovarianceStamped()
        self.pos.header.frame_id = "map"

        # Check if first publish
        self.it = 0

        # Get Amcl pose
        self.amclpose = [0,0,0]

        # Marker pose
        self.markerpose =[0,0,0]


        # Do something about the time
        self.posePublisher = rospy.Publisher("Pose", PoseWithCovarianceStamped,
                                             queue_size=10)  # pulish of position+orioentation to topic poses, type Poses
        self.particlesPublisher = rospy.Publisher("PoseArrays", PoseArray,
                                                  queue_size=10)  # publisher of particles in poseArray
        rospy.Subscriber("/RosAria/pose",Odometry, self.odomCallback)  # subscriber for odometry to be used for motionupdate
        rospy.Subscriber("/scan",LaserScan, self.sensorCallback)  # subscribe to kinect scan for the sensorupdate
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amclCallback)
        #rospy.spin()


    def odomCallback(self, msg):
        l = self.particleFilter.getOdom(msg)
        self.newodom = True

    def amclCallback(self,msg):
        self.amclpose[0] = msg.pose.pose.position.x
        self.amclpose[1] = msg.pose.pose.position.y
        (t0, t1, theta) = tf.transformations.euler_from_quaternion((0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.amclpose[2] = theta

    def publishPoseArray(self):
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
        x = []
        y=[]
        theta=[]
        #x=0
        #y=0
        #theta=0
        for particle in self.particleFilter.particles:
            msg = self.particleFilter.createPose(particle)
            pa.poses.append(msg)
            x.append(particle.x)
            y.append(particle.y)
            theta.append(particle.theta)

            #FOR CALCULATING THE MEAN FOR ESTIMATING POSE
            #x = x+ particle.x/self.particleFilter.nParticles
            #y = y+particle.y/self.particleFilter.nParticles
            #theta = theta + particle.theta/self.particleFilter.nParticles

        self.markerpose[0]=median(x)
        self.markerpose[1]=median(y)
        self.pos.pose.pose.position.x =self.markerpose[0]
        self.pos.pose.pose.position.y = self.markerpose[1]
        thetav = median(theta)
        self.markerpose[2]=thetav

        quaternion = tf.transformations.quaternion_from_euler(0, 0, thetav)
        self.pos.pose.pose.orientation.z =quaternion[2]
        self.pos.pose.pose.orientation.w = quaternion[3]
        if self.it ==0:
            self.particlesPublisher.publish(pa)
            self.posePublisher.publish(self.pos)
        else:
            self.particlesPublisher.publish(pa)
            self.posePublisher.publish(self.pos)




    def sensorCallback(self, msg):
        # Laser min/max angle and range are constant and will only be set the first time
        if (self.particleFilter.laser_max_range == 0):
            self.particleFilter.laser_min_angle = msg.angle_min
            self.particleFilter.laser_max_angle = msg.angle_max
            self.particleFilter.laser_min_range = msg.range_min
            self.particleFilter.laser_max_range = msg.range_max
        self.particleFilter.scanMsg = msg
        self.newscan = True

    def mapCallback(self, msg):
        self.particleFilter.createMap(msg)

    def runmcl(self):
        while not rospy.is_shutdown():
            if self.it ==0:
                self.publishPoseArray()
            if self.newodom and self.newscan:
                self.newodom = False
                self.newscan = False
                self.particleFilter.getU()
                for particle in self.particleFilter.particles:
                    self.particleFilter.predictParticlePose(particle)
                self.particleFilter.lastOdom = self.particleFilter.Odom

                self.particleFilter.weightUpdate(self.particleFilter.scanMsg)

                self.publishPoseArray()

                erroramcl= sqrt((self.amclpose[0]-self.markerpose[0])**2 +(self.markerpose[1]- self.amclpose[1])**2)
                self.errorlist.append(erroramcl)
                self.timelist.append(time.time())

                #error = sqrt((self.endx-11.9)**2 + (self.endy-15.45)**2)
                #print("error")
                #print(error)
        #print(self.errorlist)
        #print(self.timelist)
        plt.plot(self.timelist,self.errorlist)
        plt.ylabel("Error [m]")
        plt.xlabel("Time [s]")
        plt.title("MCL vs AMCL")
        plt.grid(True)
        plt.show()


#x=11.9
#y=15.45


if __name__ == "__main__":
    monteCarlo = MCL()
    monteCarlo.runmcl()
    print("avsluttende koordinater og avstand2")
    print(monteCarlo.endx, monteCarlo.endy)
