#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random, choices
import random
from likelihood_field import LikelihoodField
import time

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(n, p):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """

    # n = num_particles, p = weights
    # get indices of particles to choose based on weights    
    
    indices = random.choices(range(n), weights = p, k = n)
    return indices

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """

    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w


class ParticleFilter:

    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and likelihood field
        self.map = OccupancyGrid()
        self.likelihood_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 2500

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        time.sleep(3)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True


    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        # initialize particle cloud by randomly sampling from possible space in maze
        """our code starts here""" 
        # get the bounds of unoccupied space using get_obstacle_bounding_box from likelihood field
        ((x_lower, x_upper), (y_lower, y_upper)) = self.likelihood_field.get_obstacle_bounding_box()

        # sample (x,y) from bounding box and orientation uniformly
        xs = (x_upper - x_lower) * random_sample(size = self.num_particles) + x_lower
        ys = (y_upper - y_lower) * random_sample(size = self.num_particles) + y_lower
        thetas = np.random.choice(range(360), size = self.num_particles)

        # append particle to cloud
        for i in range(self.num_particles):
            p = Pose()

            p.position = Point()
            p.position.x = xs[i]
            p.position.y = ys[i]

            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, thetas[i]) 
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            self.particle_cloud.append(Particle(p, 1/(self.num_particles)))
        """our code ends here"""
        
        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0

        """our code starts here"""
        weight_sum = 0.0

        # get the sum of all particle weights
        for particle in self.particle_cloud:
            weight_sum += particle.w
        
        # normalize each weight based on sum
        for particle in self.particle_cloud:
            particle.w /= weight_sum      
        """our code ends here"""


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)


    def resample_particles(self):
        # resample of particle cloud with replacement based on weights
        
        """our code starts here"""
        # get list of weights
        weights = []
        for p in self.particle_cloud:
            weights.append(p.w)
        
        indices = draw_random_sample(self.num_particles, weights)
        
        # we need to create new cloud of particles from samples to ensure that 
        # there is no reference to the same particle in the previous cloud
        new_cloud = []

        for i in indices:
            old_particle = self.particle_cloud[i]
            new_particle = Particle(
                pose = Pose(
                    Point(
                        old_particle.pose.position.x,
                        old_particle.pose.position.y, 
                        0),
                    Quaternion(
                        old_particle.pose.orientation.x,
                        old_particle.pose.orientation.y,
                        old_particle.pose.orientation.z,
                        old_particle.pose.orientation.w,
                    )
                ), w = old_particle.w)

            new_cloud.append(new_particle)
        
        self.particle_cloud = new_cloud
        """our code ends here"""


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if len(self.particle_cloud) > 0:

            # check to see if we've moved far enough to perform an update
            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):
                
                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()

                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose


    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate

        ''' our code here'''        
        # initialize sum for all Pose parameters
        xp_mean = 0.0
        yp_mean = 0.0
        x_angle = 0.0
        y_angle = 0.0

        # sum parameters for all particles in our cloud
        for p in self.particle_cloud:
            xp_mean += p.pose.position.x
            yp_mean += p.pose.position.y
            x_angle += np.cos(get_yaw_from_pose(p.pose))
            y_angle += np.sin(get_yaw_from_pose(p.pose))
            
        # calculate the averages
        # calculate the average angle as arctan(sum(sin(angles)/cos(angles)))
        n = len(self.particle_cloud)
        xp_mean /= n
        yp_mean /= n
        angle_mean = np.arctan2(y_angle, x_angle)
        q = quaternion_from_euler(0, 0, angle_mean)

        self.robot_estimate = Pose(
            Point(xp_mean, yp_mean, 0),
            Quaternion(q[0], q[1], q[2], q[3]))

    
    def update_particle_weights_with_measurement_model(self, data):
        # update the particle weights based on likelihood field obtained from comparing to LiDAR measurements

        """our code starts here"""
        cardinal_directions_idxs = [0,45,90,135,180,225,270,315]
        z_max = 3.5 # maximum range of laser measurements

        # compute the importance weights (w) for all particles
        # using the likelihood field measurement algorithm

        # for each particle 
        for p in self.particle_cloud:
            p.w = 1

            # for each measurement in a certain direction
            for d in cardinal_directions_idxs:
                # convert inf and nan readings to max range
                m = data.ranges[d]
                if np.isfinite(m) == False:
                    m = z_max 
                
                # get angular.z by converting quaternion to yaw
                theta = get_yaw_from_pose(p.pose)

                # x_zkt = x + x_k,sens * cos(theta) - y_k,sens * sin(theta) + z_kt * cos(theta + theta_k,sens)
                # y_zkt = y + y_k,sens * cos(theta) - x_k,sens * sin(theta) + z_kt * sin(theta + theta_k,sens)
                # currently assuming x_k,sens, y_ksens, and theta_k,sens = 0 i.e. sensor at robot center
                x_zkt = p.pose.position.x + m * np.cos(theta + (d * np.pi/180)) 
                y_zkt = p.pose.position.y + m * np.sin(theta + (d * np.pi/180)) 
                dist = self.likelihood_field.get_closest_obstacle_distance(x_zkt, y_zkt)
                
                # if min distance is inf or NaN (outside maze) we set the weight to zero
                if np.isfinite(dist) == False:
                    p.w *= 0.0
                # otherwise calculate probability as in the algorithm
                else:
                    p.w *= compute_prob_zero_centered_gaussian(dist, 0.1)
        """our code ends here"""
        

    def update_particles_with_motion_model(self):

        # move particles based on relative odometry of the robot

        """our code starts here"""
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # rotate -> move straight -> rotate to final position
        x_diff = curr_x - old_x
        y_diff = curr_y - old_y

        dist = np.sqrt(x_diff**2 + y_diff**2) # linear distance traveled
        rot1 = np.arctan2(y_diff, x_diff) - old_yaw # first rotation
        rot2 = curr_yaw - old_yaw - rot1 # second rotation        

        # for each particle
        for p in self.particle_cloud:
            # first rotation
            p_yaw = get_yaw_from_pose(p.pose)
            p_yaw += rot1

            # translation
            p.pose.position.x += dist * np.cos(p_yaw) + random.gauss(0, 0.01)
            p.pose.position.y += dist * np.sin(p_yaw) + random.gauss(0, 0.01)

            # second rotation
            p_yaw += rot2+ random.gauss(0, 0.01)
            
            q = quaternion_from_euler(0.0, 0.0, p_yaw)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
        """our code ends here"""


if __name__=="__main__":
    pf = ParticleFilter()
    rospy.spin()
