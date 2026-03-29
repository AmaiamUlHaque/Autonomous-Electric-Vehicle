#!/usr/bin/env python
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

        # Read the Wall-Following controller paramters form params.yaml
        # ...
    
        # Subscrbie to LiDAR scan Wheel Odometry topics. This is to read the LiDAR scan data and vehicle actual velocity
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        # Create a publisher for the Drive topic
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        

     # The LiDAR callback function is where you read LiDAR scan data as it becomes availble and compute the vehile veloicty and steering angle commands
    
    def lidar_callback(self, data):      

      # Exttract the parameters of two walls on the left and right side of the vehicles. Referrring to Fig. 1 in the lab instructions, these are al, bl, thethal, ... 
      # ...
      # Compute the steering angle command to maintain the vehicle in the middle of left and and right walls
      # ...  
 
      # Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
      #  ...     

      # Publish steering angle and velocity commnads to the Drive topic
      # ...


    # The Odometry callback reads the actual vehicle velocity from VESC. 
    
    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
