#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import time
import numpy as np

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
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)
        # ...
        
        # Add your publisher for Drive topic here
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = rospy.Time.now()
        # ...

        # Initialize variables as needed 
        # TO INITIALISE
        front_angle = 
        angle_offset_a = 
        angle_offset_b = 
        width = getparams(width)
        wheel_radius = get params (wheel_radius , 130)
        lidar to front = get params (lidar to front, 270.17)
        w = width /2
        l = lidar to front + wheel radius
        angle_from_lidar_to_front_wheel = atan(w/l)
        # ...

        # ADD PRINT MESSAGES FOR VALIDATION PURPOSES
		# ...ooopsies, im too lazy, ill do it soon?
        #...


    # Optional function to pre-process LiDAR by considering only those LiDAR returns within a FOV in front of the vehicle
    # input: raw lidar ranges
    # output: fov lidar ranges
    def preprocess_lidar(self, ranges):
        ranges = np.clip(ranges[self.right_fov_index: self.left_fov_index],0.0,self.max_lidar_range)
        return ranges
    # ...            
             


    # Optional function to find the the maximum gap in fron the vehicle 
    # input: fov ranges
    # output: processed fov ranges , best gap start index , best gap index length
    def find_max_gap(self, ranges):
        # keeps track of gaps and the biggest one
        gap_start = 0
        gap_length = 0
        max_gap_start = gap_start
        max_gap_length = gap_length

        for i in range(len(ranges)):
            if(abs(ranges[i]) < self.safe_distance): # not a gap
                ranges[i] = 0.0

                if (gap_length > max_gap_length): #if gap better than prev best gap
                    max_gap_start = gap_start
                    max_gap_length = gap_length
                
                # reset prev gap
                gap_start = i+1
                gap_length = 0

            else: # (ranges[i] > 0): # a gap
                gap_length += 1

        # check just in case for final
        if (gap_length > max_gap_length): #if gap better than prev best gap
            max_gap_start = gap_start
            max_gap_length = gap_length

        # return in order for find_best_line function
        return (max_gap_start, max_gap_length, ranges)
    # ...
    
    
    # Optional function to find the best direction of travel
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # output: none --> self.best_angle
    def find_best_angle(self, gap_start, gap_length, proc_ranges):

        num_sum = 0
        denom_sum = 0

        for i in range(gap_length):
            j = gap_start + i
            distance = proc_ranges[j]
            angle = self.idxToRad(j+self.fov_start_idx)
            num_sum += distance*angle
            denom_sum += distance
        
        if (denom_sum == 0):
            self.best_angle = 0.0
        else:
            self.best_angle = num_sum/denom_sum
    # ...    


    # Optional function to pre-process walls by considering only those LiDAR returns within a FOV in front of the vehicle
    # input: 
    # output: 
    def preprocess_walls(self,ranges): 
        best_index = int(self.degToIdx(self.best_angle))

        
        self.al_index = (best_index + 2*self.angle_offset_a) % self.scan_beams
        self.bl_index = (best_index + 2*self.angle_offset_b) % self.scan_beams

        self.ar_index = (best_index - 2*self.angle_offset_b) % self.scan_beams
        self.br_index = (best_index - 2*self.angle_offset_a) % self.scan_beams


        if self.al_index < self.bl_index:
            left_obstacles = ranges[self.al_index : self.bl_index]
        else:
            left_obstacles = ranges[self.al_index :] + ranges[: self.bl_index]


        if self.br_index < self.ar_index:
            right_obstacles = ranges[self.br_index : self.ar_index]
        else:
            right_obstacles = ranges[self.br_index :] + ranges[: self.ar_index]
        
        
        return left_obstacles,right_obstacles


    # Optional function to set up and solve the optimization problem for parallel virtual barriers 
    def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, alpha):
        epsilon = 1e-3
        G = np.diag([1.0, 1.0, 1e-4])
        a = np.zeros(3)
        
        left_pts = []
        for i in range(len(left_obstacles)):
            left_angle = self.idxToRad(self.al_index+i) - math.pi #relative to upper vertical axis
            y = left_obstacles[i]*math.sin(left_angle)
            x = left_obstacles[i]*math.cos(left_angle)
            left_pts.append([-x, -y, -1])

        C_left = np.array(left_pts) 
        b_left = np.ones(len(left_pts))

        right_pts = []
        for j in range(len(right_obstacles)): 
            right_angle = self.idxToRad(self.br_index+j) - math.pi/2.0  #relative to right horizontal axis
            y = -right_obstacles[j]*math.cos(right_angle)
            x = right_obstacles[j]*math.sin(right_angle)
            right_pts.append([x, y, 1])

        C_right = np.array(right_pts) 
        b_right = np.ones(len(right_pts))

        C_b = np.array([
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0]
        ])
        b_b = np.array([-1 + epsilon, -1 + epsilon])

        # combine all constraints
        C_total = np.vstack([C_right,C_left, C_b])
        b_total = np.concatenate((b_right, b_left, b_b))
        
        solution = solve_qp(G, a, C_total.T, b_total,0)[0]
        
        w = solution[0:2]
        b = solution[2]

        wr = w / (b - 1.0 + epsilon)  # added a small esp to avoid division by zero
        wl = w / (b + 1.0 + epsilon)

        # Compute the distance to the walls
        dl = 1.0 / math.sqrt(wl[0]**2 + wl[1]**2)
        dr = 1.0 / math.sqrt(wr[0]**2 + wr[1]**2)
        
        return wl, wr, dl, dr
     
    # ...
     

    # This function is called whenever a new set of LiDAR data is received; bulk of your controller implementation should go here 
    def lidar_callback(self, data):   
        
        # I forgot what the LaserScan fields (ie. lidar data params) oopsies, so here they are:
            # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html:
                # Header header            # timestamp in the header is the acquisition time of the first ray in the scan.
                # float32 angle_min        # start angle of the scan [rad]
                # float32 angle_max        # end angle of the scan [rad]
                # float32 angle_increment  # angular distance between measurements [rad]
                # float32 time_increment   # time between measurements [seconds]
                # float32 scan_time        # time between scans [seconds]
                # float32 range_min        # minimum range value [m]
                # float32 range_max        # maximum range value [m]
                # float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
                # float32[] intensities    # intensity data [device-specific units].   

        # Pre-process LiDAR data as necessary
        fov_ranges = self.preprocess_lidar(data.ranges)
        # ... 
        
        # Find the widest gap in front of vehicle
        max_gap_start, max_gap_length, processed_ranges = self.find_max_gap(fov_ranges)
        # ...
        
        # Find the Best Direction of Travel
        self.best_angle = self.find_best_angle( max_gap_start, max_gap_length, processed_ranges)
        # ...

        # Set up the QP for finding the two parallel barrier lines
        obstacles_l , obstacles_r = self.preprocess_walls(data.ranges)
        # ...

        # Solve the QP problem to find the barrier lines parameters w,b
        wl, wr, dl, dr = self.getWalls(obstacles_l, obstacles_r, self.al_index, self.br_index, 0)
        # ...

        # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
        # normalise wall vectors
        wl_norm = wl / np.linalg.norm(wl)
        wr_norm = wr / np.linalg.norm(wr)
        # distance to walls derivative
        self.dl_dot = np.dot(vehicle_velocity, self.wl_norm) 
        self.dr_dot = np.dot(vehicle_velocity, self.wr_norm)
        # ...
        
        # Compute the steering angle command
        self.steering_angle = np.clip(self.steering_angle,-self.steering_angle_max,self.steering_angle_max)
		# ...
            
        # Find the closest obstacle point in a narrow field of view in fronnt of the vehicle and compute the velocity command accordingly    
        self.vel_command = np.clip(self.vel_command,0.0,self.max_speed)
        # ...
            
        # Publish the steering and speed commands to the drive topic
        self.drive_msg.header.stamp = rospy.Time.now()
		self.drive_msg.drive.speed = self.vel_command
		self.drive_msg.drive.steering_angle = self.steering_angle
		self.drive_pub.publish(self.drive_msg)
        # ...


    # Odometry callback 
    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.z = odom_msg.pose.pose.orientation.z # ONLY need z & y, do trig inside of here and update yaw
        self.w = odom_msg.pose.pose.orientation.w
        self.yaw = 2* math.atan2(self.z, self.w)
    
    
    # UNIT CONVERSIONS HELPER FUNCTIONS
    def degToRad(self, deg):
        return deg*2.0*math.pi/360.0
    
    def radToDeg(self, rad):
        return rad*360.0/(2.0*math.pi)
    
    def degToIdx(self, deg):
        return deg*self.scan_beams/360.0
    
    def idxToDeg(self, idx):
        return idx*360.0/self.scan_beam
    
    def radToIdx(self, rad):
        return int(rad*self.scan_beams/(2.0*math.pi))
    
    def idxToRad(self, idx):
        return int(idx*2.0*math.pi/self.scan_beams)
    
        


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)