#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class OccupancyGridMap {
private:
    // A ROS node
    ros::NodeHandle n;
    
    // Topics & Subs, Pubs
    // Read parameters from params.yaml
    std::string lidarscan_topic;
    std::string odom_topic = "/odom_imu";
    
    ros::Time t_prev;
    double max_lidar_range;
    int scan_beams;
    
    // Read the map parameters from *.yaml file
    // ....
    
    nav_msgs::OccupancyGrid map_occ_grid_msg;
    
    // Initialize the map meta info in the Occupancy Grid Message, e.g., frame_id, stamp, resolution, width, height, etc.
    // ...
    
    // Initialize the cell occupancy probabilities to 0.5 (unknown) with all cell data in Occupancy Grid Message set to unknown
    
    // Subscribe to Lidar scan and odometry topics with corresponding lidar_callback() and odometry_callback() functions
    ros::Subscriber lidar_sub;
    ros::Subscriber odom_sub;
    
    // Create a publisher for the Occupancy Grid Map
    ros::Publisher map_pub;

public:
    OccupancyGridMap() : n("~"), t_prev(ros::Time::now()) {
        // Read parameters from params.yaml
        n.getParam("scan_topic", lidarscan_topic);
        n.getParam("scan_range", max_lidar_range);
        n.getParam("scan_beams", scan_beams);
        
        // Read the map parameters from *.yaml file
        // ....
        
        // Initialize the map meta info in the Occupancy Grid Message, e.g., frame_id, stamp, resolution, width, height, etc.
        // ...
        
        // Initialize the cell occupancy probabilities to 0.5 (unknown) with all cell data in Occupancy Grid Message set to unknown
        
        // Subscribe to Lidar scan and odometry topics with corresponding lidar_callback() and odometry_callback() functions
        lidar_sub = n.subscribe(lidarscan_topic, 1, &OccupancyGridMap::lidarCallback, this);
        odom_sub = n.subscribe(odom_topic, 1, &OccupancyGridMap::odomCallback, this);
        
        // Create a publisher for the Occupancy Grid Map
        map_pub = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid_map", 1);
    }
    
    // lidar_callback() uses the current LiDAR scan and Wheel Odometry data to update and publish the Grid Occupancy map
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& data) {
        // ...
        
        // Publish to map topic
        map_occ_grid_msg.header.stamp = ros::Time::now();
        map_pub.publish(map_occ_grid_msg);
    }
    
    // odom_callback() retrieves the wheel odometry data from the published odom_msg
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // ...
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "occupancygridmap");
    OccupancyGridMap ogm;
    ros::Duration(0.1).sleep();
    ros::spin();
    return 0;
}