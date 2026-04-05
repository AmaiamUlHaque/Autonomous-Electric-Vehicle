#!/usr/bin/env python

from __future__ import print_function
import sys
import math
import numpy as np
import time

from numpy import array, dot
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
        self.wheelbase = rospy.get_param("~wheelbase")
        self.max_speed = rospy.get_param("~max_speed")
        self.max_steering_angle = rospy.get_param("~max_steering_angle")
        self.k_p = rospy.get_param("~k_p")
        self.k_d = rospy.get_param("~k_d")
        self.safe_distance = rospy.get_param("~safe_distance")
        self.right_beam_angle = rospy.get_param("~right_beam_angle")
        self.left_beam_angle = rospy.get_param("~left_beam_angle")
        self.vehicle_velocity = rospy.get_param("~vehicle_velocity")
        self.stop_distance = rospy.get_param("~stop_distance")
        self.stop_distance_decay = rospy.get_param("~stop_distance_decay")
        self.heading_beam_angle = rospy.get_param("~heading_beam_angle")
        self.max_lidar_range = rospy.get_param("~scan_range")
        self.fov_angle = math.radians(140)  # +-70 degrees
        self.epsilon = 1e-6  # Small number for constraints
        self.free_space_max_weight = self.max_lidar_range  # 12.0 meters
        
        # Topic names
        lidarscan_topic = rospy.get_param("~scan_topic")
        odom_topic = rospy.get_param("~odom_topic")
        drive_topic = rospy.get_param("~drive_topic")
        
        # Add your subscribers for LiDAR scan and Odomotery here
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        # ...
        
        # Add your publisher for Drive topic here
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = rospy.Time.now()
        # ...
        
        # Initialize variables as needed 
        self.vel = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.steering_angle = 0.0
        self.vel_command = 0.0
        
        self.last_ranges = None
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        
        # ADD PRINT MESSAGES FOR VALIDATION PURPOSES
        print("GapBarrier Node Initialized")
        print("  k_p: " + str(self.k_p) + ", k_d: " + str(self.k_d))
        print("  max_speed: " + str(self.max_speed) + ", max_steering: " + str(self.max_steering_angle))
        print("  safe_distance: " + str(self.safe_distance))
        print("  free_space_max_weight: " + str(self.free_space_max_weight))
        #...

    # Optional function to pre-process LiDAR by considering only those LiDAR returns within a FOV in front of the vehicle
    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        """
        Pre-process LiDAR data:
        1. Keep only points within FOV in front of vehicle
        2. Set obstacles within safe distance to 0 (danger zones)
        3. Set FREE SPACE to max_lidar_range value so it dominates theta_des
        """
        # Find indices for FOV (+-70 degrees around forward direction)
        # Forward direction is angle=0 in base_link frame
        # LiDAR frame has 0 pointing backward, so we need to offset
        forward_angle = 0  # In base_link frame, forward is 0
        
        # Convert FOV to indices
        half_fov = self.fov_angle / 2
        left_fov_angle = forward_angle + half_fov
        right_fov_angle = forward_angle - half_fov
        
        # Find indices
        left_idx = int((left_fov_angle - angle_min) / angle_increment)
        right_idx = int((right_fov_angle - angle_min) / angle_increment)
        
        # Ensure indices are within bounds
        left_idx = min(left_idx, len(ranges) - 1)
        right_idx = max(right_idx, 0)
        
        # Extract FOV ranges
        if right_idx < left_idx:
            fov_ranges = list(ranges[right_idx:left_idx])
        else:
            fov_ranges = list(ranges[left_idx:right_idx])
        
        # Convert to numpy array
        proc_ranges = np.array(fov_ranges)
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_lidar_range)
        
        # Create array for free space weights (used for theta_des calculation)
        free_space_weights = np.ones_like(proc_ranges) * self.free_space_max_weight
        
        # Process each point
        for i in range(len(proc_ranges)):
            range_val = proc_ranges[i]
            
            # Case 1: Obstacle within safe distance -> DANGER (zero out)
            if range_val < self.safe_distance and range_val > 0:
                free_space_weights[i] = 0.0  # Obstacle - don't go here
                proc_ranges[i] = 0.0  # Mark as obstacle in original array
            
            # Case 2: Valid obstacle beyond safe distance -> Keep original value as weight
            elif range_val >= self.safe_distance and range_val <= self.max_lidar_range:
                free_space_weights[i] = range_val  # Obstacles have their distance as weight
                # proc_ranges[i] stays as range_val for gap finding
            
            # Case 3: No return (0) or beyond max range -> FREE SPACE
            elif range_val == 0 or range_val > self.max_lidar_range:
                free_space_weights[i] = self.free_space_max_weight  # High weight for free space
                proc_ranges[i] = self.free_space_max_weight  # Set to high value for gap finding
        
        return proc_ranges, free_space_weights, right_idx, left_idx

    # Optional function to find the the maximum gap in fron the vehicle 
    def find_max_gap(self, proc_ranges):
        # Find the largest gap of consecutive non-zero LiDAR returns
        max_gap_size = 0
        max_gap_start = 0
        max_gap_end = 0
        
        current_start = 0
        current_size = 0
        
        for i in range(len(proc_ranges)):
            if proc_ranges[i] > 0:  # Free space
                if current_size == 0:
                    current_start = i
                current_size = current_size + 1
            else:  # Obstacle
                if current_size > max_gap_size:
                    max_gap_size = current_size
                    max_gap_start = current_start
                    max_gap_end = i - 1
                current_size = 0
        
        # Check at the end
        if current_size > max_gap_size:
            max_gap_size = current_size
            max_gap_start = current_start
            max_gap_end = len(proc_ranges) - 1
            
        return max_gap_start, max_gap_end
        
    # Optional function to find the best direction of travel
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns desired angle (theta_des) in radians
    def find_best_point(self, start_i, end_i, proc_ranges, free_space_weights, angle_min, angle_increment, original_indices_offset):
        """
        Find the best direction of travel using weighted average method.
        Uses FREE SPACE WEIGHTS where free space has high value (max_lidar_range)
        and obstacles have lower values (their actual distance or 0 if too close).
        This makes the vehicle aim for OPEN SPACE, not obstacles!
        """
        if start_i >= end_i:
            return 0.0, None
            
        # Weighted average of all beam angles in the gap (Equation 22)
        # theta_des = sum(weight_i * theta_i) / sum(weight_i)
        sum_weight = 0.0
        sum_weight_theta = 0.0
        
        for i in range(start_i, end_i + 1):
            # Use free_space_weights for weighting (not proc_ranges!)
            weight = free_space_weights[i]
            
            if weight > 0:  # Only consider positive weights
                # Convert local index to global LiDAR index
                global_idx = i + original_indices_offset
                theta = angle_min + global_idx * angle_increment
                
                # Convert to base_link frame (LiDAR has 0 pointing backward)
                # In base_link, forward is 0, left is positive
                # LiDAR typically has 0 pointing backward, pi/2 pointing left
                # So we need to adjust: base_link_angle = lidar_angle - pi
                theta_base = theta - math.pi
                
                # Normalize to [-pi, pi]
                if theta_base > math.pi:
                    theta_base = theta_base - 2 * math.pi
                if theta_base < -math.pi:
                    theta_base = theta_base + 2 * math.pi
                
                sum_weight = sum_weight + weight
                sum_weight_theta = sum_weight_theta + (weight * theta_base)
        
        if sum_weight > 0:
            theta_des = sum_weight_theta / sum_weight
        else:
            theta_des = 0.0
            
        # Find the furthest point for reference (used for visualization/debugging)
        max_range = 0
        best_idx = start_i
        for i in range(start_i, end_i + 1):
            if proc_ranges[i] > max_range:
                max_range = proc_ranges[i]
                best_idx = i
        
        return theta_des, (max_range, theta_des)

    # Optional function to set up and solve the optimization problem for parallel virtual barriers 
    def get_obstacle_points(self, ranges, angle_min, angle_increment, theta_des, right_idx, left_idx):
        # Separate obstacle points into left and right sides based on theta_des.
        # Returns points for QP formulation.
        left_points = []   # Green points (should satisfy w^T p + s <= -1)
        right_points = []  # Red points (should satisfy w^T p + s >= 1)
        
        # Define beam angles relative to desired direction
        # Left side: angles > theta_des (within 90 degrees)
        # Right side: angles < theta_des (within 90 degrees)
        
        for i in range(right_idx, left_idx):
            if i >= len(ranges):
                break
                
            range_val = ranges[i]
            if range_val <= 0 or range_val >= self.max_lidar_range:
                continue
                
            # Calculate angle in base_link frame
            theta = angle_min + i * angle_increment
            theta_base = theta - math.pi
            if theta_base > math.pi:
                theta_base = theta_base - 2 * math.pi
            if theta_base < -math.pi:
                theta_base = theta_base + 2 * math.pi
            
            # Calculate point coordinates in base_link frame
            px = range_val * math.cos(theta_base)
            py = range_val * math.sin(theta_base)
            
            # Determine if point is left or right of desired direction
            # Also limit to +-90 degrees from desired direction
            angle_diff = theta_base - theta_des
            if angle_diff > math.pi:
                angle_diff = angle_diff - 2 * math.pi
            if angle_diff < -math.pi:
                angle_diff = angle_diff + 2 * math.pi
                
            if abs(angle_diff) > math.pi / 2:
                continue  # Too far to the side
                
            if angle_diff > 0:  # Left side
                left_points.append([px, py])
            else:  # Right side
                right_points.append([px, py])
                
        return np.array(left_points), np.array(right_points)
     
    def solve_qp_barriers(self, left_points, right_points):
        # Solve the QP problem to find parallel barrier lines.
        # minimize 0.5 * w^T w
        # subject to:
        #     w^T p_i + s >= 1 for right points
        #     w^T p_j + s <= -1 for left points
        #     -1 + epsilon <= s <= 1 - epsilon
        
        n_left = len(left_points)
        n_right = len(right_points)
        
        if n_left == 0 and n_right == 0:
            # No obstacles, return default barriers
            return np.array([1.0, 0.0]), 0.0
        if n_left == 0:
            # Only right points
            n_left = 1
            left_points = np.array([[100.0, 100.0]])  # Far away
        if n_right == 0:
            # Only left points
            n_right = 1
            right_points = np.array([[100.0, 100.0]])  # Far away
            
        # Number of decision variables: w1, w2, s (3 variables)
        n_vars = 3
        
        # G matrix (Hessian): minimize 0.5 * x^T G x
        # x = [w1, w2, s]
        G = np.eye(n_vars)
        G[2, 2] = 1e-4  # Small penalty on s to ensure positive definiteness
        
        # a vector (linear term, set to 0 for minimization of quadratic only)
        a = np.zeros(n_vars)
        
        # Build constraints: C^T x >= b
        # For right points: w^T p_i + s >= 1  => [p_ix, p_iy, 1] dot [w1, w2, s] >= 1
        # For left points: w^T p_j + s <= -1 => -[p_jx, p_jy, 1] dot [w1, w2, s] >= 1
        
        n_constraints = n_right + n_left + 2  # +2 for s bounds
        C = np.zeros((n_vars, n_constraints))
        b = np.zeros(n_constraints)
        
        constraint_idx = 0
        
        # Right points constraints
        for i in range(n_right):
            C[0, constraint_idx] = right_points[i, 0]
            C[1, constraint_idx] = right_points[i, 1]
            C[2, constraint_idx] = 1.0
            b[constraint_idx] = 1.0
            constraint_idx = constraint_idx + 1
        
        # Left points constraints (multiply by -1 to convert <= to >=)
        for i in range(n_left):
            C[0, constraint_idx] = -left_points[i, 0]
            C[1, constraint_idx] = -left_points[i, 1]
            C[2, constraint_idx] = -1.0
            b[constraint_idx] = 1.0
            constraint_idx = constraint_idx + 1
        
        # s bounds: s >= -1 + epsilon  =>  [0, 0, 1] dot x >= -1 + epsilon
        C[2, constraint_idx] = 1.0
        b[constraint_idx] = -1.0 + self.epsilon
        constraint_idx = constraint_idx + 1
        
        # s bounds: s <= 1 - epsilon  => -[0, 0, 1] dot x >= -1 + epsilon
        C[2, constraint_idx] = -1.0
        b[constraint_idx] = -1.0 + self.epsilon
        constraint_idx = constraint_idx + 1
        
        try:
            # Solve QP
            x = solve_qp(G, a, C, b, meq=0)[0]
            w = np.array([x[0], x[1]])
            s_val = x[2]
            return w, s_val
        except Exception as e:
            rospy.logwarn("QP solver failed: " + str(e))
            # Return default barriers
            return np.array([1.0, 0.0]), 0.0

    def compute_barrier_distances_and_angles(self, w, s_val):
        # Compute distances to left and right barriers and cos(alpha) values.
        # Equations (30), (31), (33), (36) from lab manual.
        
        # Compute w_r and w_l from (31)
        # Note: Need to handle division by zero
        if abs(s_val - 1) < 1e-6:
            w_r = w * 1000  # Large distance
        else:
            w_r = w / (s_val - 1)
            
        if abs(s_val + 1) < 1e-6:
            w_l = w * 1000
        else:
            w_l = w / (s_val + 1)
        
        # Distances to barriers (30)
        norm_w_r = np.linalg.norm(w_r)
        norm_w_l = np.linalg.norm(w_l)
        
        if norm_w_r < 1e-6:
            d_r = 100.0
        else:
            d_r = 1.0 / norm_w_r
            
        if norm_w_l < 1e-6:
            d_l = 100.0
        else:
            d_l = 1.0 / norm_w_l
        
        # Unit vectors (33)
        if norm_w_r > 1e-6:
            w_hat_r = w_r / norm_w_r
        else:
            w_hat_r = np.array([1.0, 0.0])
            
        if norm_w_l > 1e-6:
            w_hat_l = w_l / norm_w_l
        else:
            w_hat_l = np.array([1.0, 0.0])
        
        # cos(alpha) values (36)
        cos_alpha_r = np.dot(np.array([0.0, 1.0]), w_hat_r)
        cos_alpha_l = np.dot(np.array([0.0, -1.0]), w_hat_l)
        
        # Rate of change of distances (34)
        d_dot_l = self.vel * w_hat_l[0]
        d_dot_r = self.vel * w_hat_r[0]
        d_lr_dot = d_dot_l - d_dot_r
        
        return d_l, d_r, d_lr_dot, cos_alpha_l, cos_alpha_r

    def compute_steering_command(self, d_lr_dot, cos_alpha_l, cos_alpha_r, d_l, d_r):
        # Compute steering angle command using equation (8).
        
        d_lr = d_l - d_r
        d_lr_des = 0.0  # Desired difference (center between barriers)
        
        # Tracking error (7)
        d_lr_tilde = d_lr - d_lr_des
        
        # Control law (8)
        denominator = self.vel**2 * (cos_alpha_r + cos_alpha_l)
        
        if abs(denominator) < 1e-6:
            steering_cmd = 0.0
        else:
            u = -self.k_p * d_lr_tilde - self.k_d * d_lr_dot
            steering_cmd = math.atan2(-self.wheelbase * u, denominator)
        
        # Clamp steering angle (11)
        steering_cmd = np.clip(steering_cmd, -self.max_steering_angle, self.max_steering_angle)
        
        return steering_cmd

    def compute_velocity_command(self, ranges, angle_min, angle_increment, theta_des):
        # Compute velocity command based on closest obstacle in front.
        # Equation (20) from lab manual.
        
        # Find closest obstacle in narrow FOV in front
        half_heading = self.heading_beam_angle / 2
        
        # Find indices for heading FOV
        left_heading = theta_des + half_heading
        right_heading = theta_des - half_heading
        
        # Convert to LiDAR angles
        left_lidar = left_heading + math.pi
        right_lidar = right_heading + math.pi
        
        # Normalize
        if left_lidar > math.pi:
            left_lidar = left_lidar - 2 * math.pi
        if right_lidar < -math.pi:
            right_lidar = right_lidar + 2 * math.pi
            
        # Find indices
        left_idx = int((left_lidar - angle_min) / angle_increment)
        right_idx = int((right_lidar - angle_min) / angle_increment)
        
        if right_idx < left_idx:
            start_idx = max(0, right_idx)
            end_idx = min(len(ranges) - 1, left_idx)
        else:
            start_idx = max(0, left_idx)
            end_idx = min(len(ranges) - 1, right_idx)
        
        # Find minimum range in heading FOV
        min_range = float('inf')
        for i in range(start_idx, end_idx + 1):
            if i < len(ranges) and ranges[i] > 0:
                if ranges[i] < min_range:
                    min_range = ranges[i]
        
        if min_range == float('inf'):
            min_range = self.safe_distance
            
        # Velocity command (20)
        # v_s^c = v_s^d * [1 - exp(-max(d_ob - d_stop, 0) / d_tau)]
        d_ob = min_range
        exponent_arg = max(d_ob - self.stop_distance, 0) / self.stop_distance_decay
        vel_cmd = self.vehicle_velocity * (1 - math.exp(-exponent_arg))
        
        # Ensure positive velocity
        vel_cmd = max(vel_cmd, 0.05)
        
        return vel_cmd

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

        # Store LiDAR parameters
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        ranges = np.array(data.ranges)
        
        # Pre-process LiDAR data as necessary
        proc_ranges, free_space_weights, right_idx, left_idx = self.preprocess_lidar(
            ranges, self.angle_min, self.angle_increment
        )
        # ... 
        
        # Find the widest gap in front of vehicle
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        # ...
        
        # Find the Best Direction of Travel using weighted average with free space weights
        theta_des, best_point = self.find_best_point(
            gap_start, gap_end, proc_ranges, free_space_weights,
            self.angle_min, self.angle_increment, right_idx
        )
        # ...

        # Set up the QP for finding the two parallel barrier lines
        left_points, right_points = self.get_obstacle_points(
            ranges, self.angle_min, self.angle_increment, 
            theta_des, right_idx, left_idx
        )
        # ...

        # Solve the QP problem to find the barrier lines parameters w,b
        w, s_val = self.solve_qp_barriers(left_points, right_points)
        # ...

        # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
        d_l, d_r, d_lr_dot, cos_alpha_l, cos_alpha_r = self.compute_barrier_distances_and_angles(w, s_val)
        # ...
        
        # Compute the steering angle command
        self.steering_angle = self.compute_steering_command(
            d_lr_dot, cos_alpha_l, cos_alpha_r, d_l, d_r
        )
        self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)
        # ...
            
        # Find the closest obstacle point in a narrow field of view in front of the vehicle and compute the velocity command accordingly    
        self.vel_command = self.compute_velocity_command(
            ranges, self.angle_min, self.angle_increment, theta_des
        )
        self.vel_command = np.clip(self.vel_command, 0.0, self.max_speed)
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
        self.yaw = 2 * math.atan2(self.z, self.w)


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)