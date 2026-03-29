#!/usr/bin/env python
from __future__ import print_function
from lib2to3.pytree import Node
import sys
import math
from tokenize import Double
import numpy as np
import time

from  numpy import array, dot
from quadprog import solve_qp
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GapBarrier:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read the algorithm parameter paramters form params.yaml
        
        # ...

 

        # Add your subscribers for LiDAR scan and Odomotery here
        # ...
        
        # Add your publisher for Drive topic here
        #...

        # Initialize varables as needed 
        #...


    # Optional function to pre-process LiDAR by considering only those LiDAR returns within a FOV in front of the vehicle;    

    def preprocess_lidar(self, ranges):
        
    # ...            
             


    # Optional function to find the the maximum gap in fron the vehicle 
    def find_max_gap(self, proc_ranges):
        
    #Optional function to find the best direction of travel
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns index of best (furthest point) in ranges
    def find_best_point(self, start_i, end_i, proc_ranges):
    # ...    



    # Optional function to set up and solve the optimization problem for parallel virtual barriers 
    def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, alpha):
     
     # ...
     

    # This function is called whenever a new set of LiDAR data is received; bulk of your controller implementation should go here 
    def lidar_callback(self, data):      

    # Pre-process LiDAR data as necessary
    # ...
    
    # Find the widest gape in front of vehicle
    # ...
    
    # Find the Best Direction of Travel
    # ...

    # Set up the QP for finding the two parallel barrier lines
    # ...

    # Solve the QP problem to find the barrier lines parameters w,b

    # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
    # ...
    
    # Compute the steering angle command
        
    # Find the closest obstacle point in a narrow field of view in fronnt of the vehicle and compute the velocity command accordingly    
    # ...
        
    # Publish the steering and speed commands to the drive topic
    # ...


    # Odometry callback 
    def odom_callback(self, odom_msg):
        # update current speed
         self.vel = odom_msg.twist.twist.linear.x


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
