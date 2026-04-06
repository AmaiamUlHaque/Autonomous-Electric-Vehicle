#!/usr/bin/env python

# THIS NAVIGATION NODE FILE WAS USED FOR ACTIVITIES 3-5
# OTHER NAVIGATION NODE FILE USED FOR ACTIVITIES 6-THE REST

from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry



class WallFollow:
    def __init__(self):

        self.x = 0 
        self.y = 0
        self.yaw = 0 

        # Read the Wall-Following controller paramters form params.yaml
        # Read paramters form params.yaml
        lidarscan_topic = rospy.get_param('~scan_topic')
        odom_topic = rospy.get_param('~odom_topic')
        drive_topic = rospy.get_param('~drive_topic')

        self.angle_al = float(rospy.get_param('~angle_al'))
        self.angle_bl = float(rospy.get_param('~angle_bl'))
        self.angle_ar = float(rospy.get_param('~angle_ar'))
        self.angle_br = float(rospy.get_param('~angle_br'))
        self.scan_beams = rospy.get_param('~scan_beams')
        self.max_lidar_range = rospy.get_param('~scan_range')
        self.length = rospy.get_param('~wheelbase')

        self.steering_angle_max = rospy.get_param('~max_steering_angle') 
        self.max_speed = rospy.get_param('~max_speed')
        self.stop_distance = rospy.get_param('~stop_distance')
        self.decay_const = rospy.get_param('~stop_distance_decay')
        self.norm_speed = rospy.get_param('~vehicle_velocity')
        self.vel = 0
        self.kd = rospy.get_param('~k_d')
        self.kp = rospy.get_param('~k_p')

        self.alpha_l = 0.0
        self.alpha_r = 0.0
        self.beta_l = 0.0
        self.beta_r = 0.0

        self.al_index = 0
        self.bl_index = 0
        self.ar_index = 0
        self.br_index = 0

        self.theta_l = self.angle_bl-self.angle_al
        self.theta_r = self.angle_ar-self.angle_br
       

        self.dlr = 0
        self.dlr_error = 0
        self.dl = 0
        self.dr = 0
        self.dlr_dot = 0 
  
        self.steering_angle = 0
        self.closest_distance = 0
        self.vel_command = 0

    
        # Subscrbie to LiDAR scan Wheel Odometry topics. This is to read the LiDAR scan data and vehicle actual velocity
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        # Create a publisher for the Drive topic
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp= rospy.Time.now()

		# ADD PRINT MESSAGES FOR VALIDATION PURPOSES
		# ...ooopsies, im too lazy, ill do it soon?
        
        

    # The LiDAR callback function is where you read LiDAR scan data as it becomes availble and compute the vehile veloicty and steering angle commands
    def lidar_callback(self, data):      

		# Exttract the parameters of two walls on the left and right side of the vehicles. 
		# Referrring to Fig. 1 in the lab instructions, these are al, bl, thethal, ... 
		
		self.al_index = int(round((self.angle_al)/data.angle_increment))%self.scan_beams
		self.bl_index = int(round((self.angle_bl)/data.angle_increment))%self.scan_beams
		self.ar_index = int(round((self.angle_ar)/data.angle_increment))%self.scan_beams
		self.br_index = int(round((self.angle_br)/data.angle_increment))%self.scan_beams

		# numpy.clip(a=input array, a_min=min value that an element of a can be, a_max=max value that an element of a can be)
		self.al = np.clip(data.ranges[self.al_index],0,self.max_lidar_range)
		self.bl = np.clip(data.ranges[self.bl_index],0,self.max_lidar_range)
		self.ar = np.clip(data.ranges[self.ar_index],0,self.max_lidar_range)
		self.br = np.clip(data.ranges[self.br_index],0,self.max_lidar_range)


		self.beta_r= math.atan2((self.ar*math.cos(self.theta_r)-self.br), (self.ar*math.sin(self.theta_r)))
		self.beta_l= math.atan2((self.al*math.cos(self.theta_l)-self.bl), (self.al*math.sin(self.theta_l)))

		self.alpha_r= self.beta_r + math.pi/2.0 -self.angle_br
		self.alpha_l= -self.beta_l+3.0*math.pi/2.0 - self.angle_bl

		self.dl=(self.bl*math.cos(self.beta_l))
		self.dr =(self.br*math.cos(self.beta_r))


		self.dlr = self.dl-self.dr # error

		
    

		error_proportional = min(abs(self.dr/self.dl) , abs(self.dl/self.dr))
		ratio = 0.1 
		# steering ratio. more error = allow more steering physically possible

		# # lINEAR RELATION
		# m = -0.95
		# b = 1
		# ratio = m*error_proportional+b

		# # EXPONENTIAL RELATION
		# b = -12/11*math.log(0.05)
		# a = math.exp(b/12)
		# ratio = a*math.exp(-b*error_proportional)
		 

		# USE IF THE LINEAR/EXPONENTIAL RELATION ISNT WORKING
		if (error_proportional >= 11/12):
			ratio = 0.05
		# elif (error_proportional >= 5/6):
		# 	ratio = 0.1
		# elif (error_proportional >= 1/3):
		# 	ratio = 0.25
		# elif (error_proportional >= 1/6):
		# 	ratio = 0.6
		else:
			ratio = 0.75
			
		if(abs(self.dlr) < 0.01): #0.01m = 1cm
			self.dlr_error= 0

		else: 
			self.dlr_error= self.dlr

		# self.dlr_error = self.dlr
		self.dlr_dot = -self.vel*math.sin(self.alpha_l)-self.vel*math.sin(self.alpha_r)

		# Compute the steering angle command to maintain the vehicle in the middle of left and and right walls
		self.steering_angle = ratio*math.atan2(self.kp*self.dlr_error*self.length+self.kd*self.dlr_dot*self.length,(self.vel**2)*(math.cos(self.alpha_r)+math.cos(self.alpha_l)))
		self.steering_angle = np.clip(self.steering_angle,-self.steering_angle_max,self.steering_angle_max)
		
 
		# Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
		self.closest_distance = min((data.ranges[350:371]))
		self.vel_command = self.norm_speed*(1- math.exp(-max(self.closest_distance-self.stop_distance, 0)/self.decay_const))
		self.vel_command = np.clip(self.vel_command,0.0,self.max_speed)


		# Publish steering angle and velocity commnads to the Drive topic
		self.drive_msg.header.stamp = rospy.Time.now()
		self.drive_msg.drive.speed = self.vel_command
		self.drive_msg.drive.steering_angle = self.steering_angle
		self.drive_pub.publish(self.drive_msg)

		# ADD PRINT MESSAGES FOR VALIDATION PURPOSES
		# ...ooopsies, im too lazy, ill do it soon?


    # The Odometry callback reads the actual vehicle velocity from VESC
    def odom_callback(self, odom_msg):
		self.vel = odom_msg.twist.twist.linear.x # update current speed
		self.x = odom_msg.pose.pose.position.x
		self.y = odom_msg.pose.pose.position.y
		self.z = odom_msg.pose.pose.orientation.z # ONLY need z & y, do trig inside of here and update yaw
		self.w = odom_msg.pose.pose.orientation.w
		self.yaw = 2* math.atan2(self.z, self.w)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)