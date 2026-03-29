#!/usr/bin/env python

import numpy as np
import sys
import cv2
import time
import rospy
import tf2_ros
import math


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class OccupancyGridMap:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read paramters form params.yaml
        lidarscan_topic =rospy.get_param('~scan_topic')
        odom_topic="/odom_imu"

        self.t_prev=rospy.get_time()
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.scan_beams=rospy.get_param('~scan_beams')
        
        # Read the map parameters from *.yaml file
        # ....

        
        self.map_occ_grid_msg = OccupancyGrid()

        # Initialize the map meta info in the Occupancy Grid Message, e.g., frame_id, stamp, resolution, width, height, etc.
        # ...

        # Initialize the cell occuopancy probabilites to 0.5 (unknown) with all cell data in Occupancy Grid Message set to unknown 

    
        # Subscribe to Lidar scan and odomery topics with corresponding lidar_callback() and odometry_callback() functions 
        # ...

        # Create a publisher for the Occupancy Grid Map

        # ...            


    # lidar_callback () uses the current LiDAR scan and Wheel Odometry data to uddate and publish the Grid Occupancy map 
    
    def lidar_callback(self, data):      

    # ...
       
        # Publish to map topic
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map_occ_grid_msg)

    # odom_callback() retrives the wheel odometry data from the publsihed odom_msg
     
    def odom_callback(self, odom_msg):

    # ...


def main(args):
    rospy.init_node("occupancygridmap", anonymous=True)
    OccupancyGridMap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
