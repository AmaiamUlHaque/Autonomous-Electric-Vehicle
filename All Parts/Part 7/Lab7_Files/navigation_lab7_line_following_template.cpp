#define _USE_MATH_DEFINES //M_PI

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h" //receive msgs from lidar
#include "sensor_msgs/Imu.h" //receive msgs from Imu


#include <tf/tf.h> //Quaternions

#include "ackermann_msgs/AckermannDriveStamped.h" //Ackermann Steering

#include "nav_msgs/Odometry.h" //Odometer

#include <string>
#include <vector>


//standard and external
#include <stdio.h>
#include <math.h>
#include <cmath> 
#include <sstream>


class WallFollow 
{
	private:
		ros::NodeHandle nf;

		//Subscriptions
		ros::Subscriber lidar;
		ros::Subscriber odom;
		
		//Publications
		ros::Publisher lidar_pub;
		ros::Publisher driver_pub;	
		
		//topics
		std::string lidarscan_topic, drive_topic, odom_topic;

	public:
		
		WallFollow(){

			nf = ros::NodeHandle("~");
		
        // Read the Wall-Following controller paramters form params.yaml
        //  ...

		//	nf.getParam( ...);

        // Subscrbie to LiDAR scan Wheel Odometry topics. This is to read the LiDAR scan data and vehicle actual velocity
			lidar = nf.subscribe("/scan",1, &WallFollow::lidar_callback, this);
			odom = nf.subscribe(odom_topic,1, &WallFollow::odom_callback, this);

        // Create a publisher for the Drive topic
			driver_pub = nf.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
		}



// The Odometry callback reads the actual vehicle velocity from VESC. 

		void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){
			vel = odom_msg->twist.twist.linear.x;
		}

     // The LiDAR callback function is where you read LiDAR scan data as it becomes availble and compute the vehile veloicty and steering angle commands


		void lidar_callback(const sensor_msgs::LaserScanConstPtr &data){
			
			
      // Exttract the parameters of two walls on the left and right side of the vehicles. Referrring to Fig. 1 in the lab instructions, these are al, bl, thethal, ... 
      // ...
      // Compute the steering angle command to maintain the vehicle in the middle of left and and right walls
      // ...  
 
      // Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
      //  ...     

      // Publish steering angle and velocity commnads to the Drive topic
      // ...
		}

};



int main(int argc, char **argv){
		ros::init(argc, argv, "navigation");
		WallFollow gb;

		while(ros::ok()){
			ros::spinOnce();
		}

}

