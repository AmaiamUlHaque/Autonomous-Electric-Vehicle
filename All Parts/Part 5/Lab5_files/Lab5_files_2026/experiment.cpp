#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <vesc_msgs/VescStateStamped.h>
#include <cmath>
#include <iostream>


class RacecarExperiment {
private:
    // A ROS node
    ros::NodeHandle n;

    // Listen to drive commands
    ros::Subscriber drive_sub;

    // Listen to vesc
    ros::Subscriber vesc_state_sub;
    ros::Subscriber servo_sub;

    //Listen to lidar
    ros::Subscriber lidar_sub;

    // Listen to mux messages
    ros::Subscriber mux_sub;

    // Publish lidar data
    ros::Publisher lidar_pub;
    
    // Publish drive data to vesc
    ros::Publisher erpm_pub;
    ros::Publisher servo_pub;
    ros::Publisher brake_pub;

    // The car state and parameters
    double desired_speed=0;double desired_steer_ang=0; double last_servo=0; double last_rpm=0;

    
    std_msgs::Float64 erpm_msg;
    std_msgs::Float64 servo_msg;
    std_msgs::Float64 brake_msg;

    std_msgs::Float64 last_servo_state; // Last recived servo state
    vesc_msgs::VescStateStamped last_vesc_state; // Last received vesc state

 
    // Odometry parameters
    double speed_to_erpm_gain, speed_to_erpm_offset,
        steering_angle_to_servo_gain, steering_angle_to_servo_offset, wheelbase;
 
    double max_acceleration, max_servo_speed, driver_smoother_rate, max_delta_servo,max_delta_rpm;
    
    ros::Timer update_command;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

public:
RacecarExperiment() {
    // Initialize the node handle
    n = ros::NodeHandle("~");
    
    // Get topic names
    std::string drive_topic, scan_topic, odom_topic, mux_topic;
    n.getParam("drive_topic", drive_topic);
    n.getParam("scan_topic", scan_topic);
    n.getParam("mux_topic", mux_topic);
    n.getParam("mux_size", mux_size);
    
    //Get vesc gains
    n.getParam("speed_to_erpm_gain", speed_to_erpm_gain);
    n.getParam("speed_to_erpm_offset", speed_to_erpm_offset);
    n.getParam("steering_angle_to_servo_gain", steering_angle_to_servo_gain);
    n.getParam("steering_angle_to_servo_offset", steering_angle_to_servo_offset);
    desired_steer_ang=steering_angle_to_servo_offset;
    last_servo=steering_angle_to_servo_offset;

    // Get car parameters
    n.getParam("wheelbase", wheelbase);
    n.getParam("max_accel", max_acceleration);
    n.getParam("max_steering_vel",max_servo_speed);
    n.getParam("driver_smoother_rate",driver_smoother_rate);

    

    // initialize mux controller
    mux_controller.reserve(mux_size);
    for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
    // Make a publisher for laser scan messages
    lidar_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);
    
    // Make a publisher for drive messages
    erpm_pub = n.advertise<std_msgs::Float64>("/commands/motor/speed", 1);
    servo_pub = n.advertise<std_msgs::Float64>("/commands/servo/position", 1);
    brake_pub = n.advertise<std_msgs::Float64>("/commands/motor/brake", 1);


    // Start a subscriber to listen to drive commands
    drive_sub = n.subscribe(drive_topic, 1, &RacecarExperiment::driver_callback, this);

  
    // Start a subscriber to listen to lidar messages  
    lidar_sub = n.subscribe("/scan2", 1, &RacecarExperiment::laser_callback, this);

    // Start a subscriber to listen to mux messages
    mux_sub = n.subscribe(mux_topic, 1, &RacecarExperiment::mux_callback, this);

    
    max_delta_servo = std::abs(steering_angle_to_servo_gain * max_servo_speed / driver_smoother_rate);
    max_delta_rpm = std::abs(speed_to_erpm_gain * max_acceleration / driver_smoother_rate);
    
    update_command = n.createTimer(ros::Duration(1.0/driver_smoother_rate), &RacecarExperiment::publish_driver_command, this);
    
    ROS_INFO("Experiment constructed.");

}



    void driver_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
        desired_speed = msg.drive.speed;
        desired_steer_ang = msg.drive.steering_angle;
        desired_speed=speed_to_erpm_gain * desired_speed - speed_to_erpm_offset;
        desired_steer_ang = steering_angle_to_servo_gain * desired_steer_ang + steering_angle_to_servo_offset;
    }



    void publish_driver_command( const ros::TimerEvent&){

        double desired_delta = desired_steer_ang-last_servo;
        double clipped_delta = std::max(std::min(desired_delta, max_delta_servo), -max_delta_servo);
        double smoothed_servo = last_servo + clipped_delta;
        last_servo = smoothed_servo;
        servo_msg.data=smoothed_servo;

        double desired_rpm = desired_speed-last_rpm;
        double clipped_rpm = std::max(std::min(desired_rpm, max_delta_rpm), -max_delta_rpm);
        double smoothed_rpm = last_rpm + clipped_rpm;
        last_rpm = smoothed_rpm; 
        erpm_msg.data=smoothed_rpm;     

        servo_pub.publish(servo_msg);
        erpm_pub.publish(erpm_msg);

    }


    void servo_callback(const std_msgs::Float64 & servo){
        last_servo_state= servo;
    }

  
    void laser_callback(const sensor_msgs::LaserScan & msg) {
      
        int n1=msg.ranges.size()/2;
        std::vector<float> scan_ranges=msg.ranges;
        std::vector<float> scan_intensities=msg.intensities;
        for (int i=0;i<n1;i++){
            scan_ranges[i]=msg.ranges[n1+i];
            scan_ranges[n1+i]=msg.ranges[i];
            scan_intensities[i]=msg.intensities[n1+i];
            scan_intensities[n1+i]=msg.intensities[i];
        }

       // Publish lidar information
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = msg.header.stamp;
        scan_msg.header.frame_id = msg.header.frame_id;
        scan_msg.angle_min = msg.angle_min+3.141592;
        scan_msg.angle_max = msg.angle_max+3.141592;
        scan_msg.angle_increment = msg.angle_increment;
        scan_msg.range_max = msg.range_max;    
        scan_msg.ranges = scan_ranges;
        scan_msg.intensities = scan_intensities;
        lidar_pub.publish(scan_msg);
    }
 


    void mux_callback(const std_msgs::Int32MultiArray & msg) {
        // Publish zero command when emergency brake is activeted
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = bool(msg.data[i]);
        }
        if (mux_controller[3]==true){
            erpm_msg.data=0.0;
            desired_speed=0.0;
            erpm_pub.publish(erpm_msg);
            }
        }
};




int main(int argc, char ** argv) {
    ros::init(argc, argv, "racecar_experiment");
    RacecarExperiment rs;
    ros::spin();
    return 0;
}
