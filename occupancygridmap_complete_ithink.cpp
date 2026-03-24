#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

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
    double resolution;
    int width;
    int height;
    double origin_x;
    double origin_y;
    
    // Occupancy Grid Map message
    nav_msgs::OccupancyGrid map_occ_grid_msg;
    
    // Store log-odds values for Bayesian updates
    std::vector<double> log_odds;
    
    // Probability parameters for occupancy grid mapping (from Lab06 instructions)
    double p_occ;  // Occupied probability (p_oc > 0.5, e.g., 0.9)
    double p_free; // Free probability (p_fr < 0.5, e.g., 0.3)
    double p_unknown; // Unknown probability (0.5)
    
    // Log-odds values for inverse sensor model
    double l_occ;
    double l_free;
    double l_unknown;
    double l_prior;
    
    // Pose from odometry
    double odom_x;
    double odom_y;
    double odom_yaw;
    bool has_odom;
    
    // Transform between base_link and laser frames
    double laser_offset_x;
    double laser_offset_y;
    double laser_offset_yaw;
    
    // Subscribe to Lidar scan and odometry topics with corresponding lidar_callback() and odometry_callback() functions
    ros::Subscriber lidar_sub;
    ros::Subscriber odom_sub;
    
    // Create a publisher for the Occupancy Grid Map
    ros::Publisher map_pub;
    

    // Helper function to convert probability to log-odds
    // l(y) = log(p(y) / (1 - p(y)))
    double probToLogOdds(double prob) {
        if (prob <= 0.0) return -std::numeric_limits<double>::infinity();
        if (prob >= 1.0) return std::numeric_limits<double>::infinity();
        return std::log(prob / (1.0 - prob));
    }
    

    // Helper function to convert log-odds to probability
    // p(y) = 1 - 1/(1 + e^(l(y)))
    double logOddsToProb(double log_odds_val) {
        return 1.0 - 1.0 / (1.0 + std::exp(log_odds_val));
    }
    

    /**
     * Convert log-odds to occupancy grid value (0-100, -1 for unknown)
     * - 100: occupied (p > p_occ threshold)
     * - 0: free (p < p_free threshold)
     * - -1: unknown
     */
    int logOddsToOccupancy(double log_odds_val) {
        double prob = logOddsToProb(log_odds_val);
        
        // Classify based on probability thresholds
        if (prob > p_occ) return 100;      // Occupied
        if (prob < p_free) return 0;       // Free
        return -1;                          // Unknown
    }
    
    
    // Convert world coordinates to grid cell indices
    // Map cell (0,0) is at origin (origin_x, origin_y) - lower left corner
    std::pair<int, int> worldToGrid(double x, double y) {
        int col = static_cast<int>((x - origin_x) / resolution);
        int row = static_cast<int>((y - origin_y) / resolution);
        
        // Check bounds
        if (row < 0 || row >= height || col < 0 || col >= width) {
            return std::make_pair(-1, -1);
        }
        return std::make_pair(row, col);
    }
    

    // Convert grid indices to linear index (row-major order)
    int gridToIndex(int row, int col) {
        return row * width + col;
    }
    

    /**
     * Convert a point from laser frame to odom frame
     * @param laser_x, laser_y: Point coordinates in laser frame
     * @return Point coordinates in odom frame
     */
    std::pair<double, double> laserToOdom(double laser_x, double laser_y) {
        // First transform from laser to base_link (static transformation)
        double base_x = laser_x * std::cos(laser_offset_yaw) - laser_y * std::sin(laser_offset_yaw) + laser_offset_x;
        double base_y = laser_x * std::sin(laser_offset_yaw) + laser_y * std::cos(laser_offset_yaw) + laser_offset_y;
        
        // Then transform from base_link to odom using current robot pose
        double odom_x = odom_x + base_x * std::cos(odom_yaw) - base_y * std::sin(odom_yaw);
        double odom_y = odom_y + base_x * std::sin(odom_yaw) + base_y * std::cos(odom_yaw);
        
        return std::make_pair(odom_x, odom_y);
    }
    
    
    /**
     * Calculate the range from laser origin to a cell center
     * @param cell_x, cell_y: Cell center coordinates in odom frame
     * @return Range to cell center in laser frame
     */
    double calculateCellRange(double cell_x, double cell_y) {
        // Transform cell center from odom to base_link
        double dx_odom = cell_x - odom_x;
        double dy_odom = cell_y - odom_y;
        double base_x = dx_odom * std::cos(odom_yaw) + dy_odom * std::sin(odom_yaw);
        double base_y = -dx_odom * std::sin(odom_yaw) + dy_odom * std::cos(odom_yaw);
        
        // Transform from base_link to laser
        double laser_x = (base_x - laser_offset_x) * std::cos(laser_offset_yaw) + (base_y - laser_offset_y) * std::sin(laser_offset_yaw);
        double laser_y = -(base_x - laser_offset_x) * std::sin(laser_offset_yaw) + (base_y - laser_offset_y) * std::cos(laser_offset_yaw);
        
        return std::sqrt(laser_x * laser_x + laser_y * laser_y);
    }
    

    /**
     * Calculate the angle of a cell relative to laser frame
     * @param cell_x, cell_y: Cell center coordinates in odom frame
     * @return Angle in radians
     */
    double calculateCellAngle(double cell_x, double cell_y) {
        // Transform cell center from odom to base_link
        double dx_odom = cell_x - odom_x;
        double dy_odom = cell_y - odom_y;
        double base_x = dx_odom * std::cos(odom_yaw) + dy_odom * std::sin(odom_yaw);
        double base_y = -dx_odom * std::sin(odom_yaw) + dy_odom * std::cos(odom_yaw);
        
        // Transform from base_link to laser
        double laser_x = (base_x - laser_offset_x) * std::cos(laser_offset_yaw) + (base_y - laser_offset_y) * std::sin(laser_offset_yaw);
        double laser_y = -(base_x - laser_offset_x) * std::sin(laser_offset_yaw) + (base_y - laser_offset_y) * std::cos(laser_offset_yaw);
        
        return std::atan2(laser_y, laser_x);
    }


public:
    OccupancyGridMap() : n("~"), t_prev(ros::Time::now()), 
                         odom_x(0.0), odom_y(0.0), odom_yaw(0.0), has_odom(false),
                         p_occ(0.9), p_free(0.3), p_unknown(0.5),
                         laser_offset_x(0.0), laser_offset_y(0.0), laser_offset_yaw(0.0) {
        // Read parameters from params.yaml
        n.getParam("scan_topic", lidarscan_topic);
        n.getParam("scan_range", max_lidar_range);
        n.getParam("scan_beams", scan_beams);
        
        // Read the map parameters from *.yaml file
        n.getParam("resolution", resolution);
        n.getParam("width", width);
        n.getParam("height", height);
        n.getParam("origin_x", origin_x);
        n.getParam("origin_y", origin_y);
        
        // Read occupancy grid parameters
        n.getParam("p_occ", p_occ);
        n.getParam("p_free", p_free);
        p_unknown = 0.5;
        
        // Read laser to base_link transformation (from static transform)
        n.getParam("laser_offset_x", laser_offset_x);
        n.getParam("laser_offset_y", laser_offset_y);
        n.getParam("laser_offset_yaw", laser_offset_yaw);
        
        // Initialize log-odds values for inverse sensor model
        l_occ = probToLogOdds(p_occ);
        l_free = probToLogOdds(p_free);
        l_unknown = probToLogOdds(p_unknown);
        l_prior = probToLogOdds(p_unknown);  // Prior is unknown (0.5)
        
        // Initialize the map meta info in the Occupancy Grid Message
        map_occ_grid_msg.header.frame_id = "odom";  // Map reference frame
        map_occ_grid_msg.info.resolution = resolution;
        map_occ_grid_msg.info.width = width;
        map_occ_grid_msg.info.height = height;
        map_occ_grid_msg.info.origin.position.x = origin_x;
        map_occ_grid_msg.info.origin.position.y = origin_y;
        map_occ_grid_msg.info.origin.position.z = 0.0;
        map_occ_grid_msg.info.origin.orientation.x = 0.0;
        map_occ_grid_msg.info.origin.orientation.y = 0.0;
        map_occ_grid_msg.info.origin.orientation.z = 0.0;
        map_occ_grid_msg.info.origin.orientation.w = 1.0;  // Zero rotation
        
        // Initialize the cell occupancy probabilities to 0.5 (unknown) with all cell data in Occupancy Grid Message set to unknown
        map_occ_grid_msg.data.resize(width * height, -1);
        
        // Initialize log-odds array with prior values (corresponding to p=0.5)
        log_odds.resize(width * height, l_prior);
        
        // Subscribe to Lidar scan and odometry topics with corresponding lidar_callback() and odometry_callback() functions
        lidar_sub = n.subscribe(lidarscan_topic, 1, &OccupancyGridMap::lidarCallback, this);
        odom_sub = n.subscribe(odom_topic, 1, &OccupancyGridMap::odomCallback, this);
        
        // Create a publisher for the Occupancy Grid Map
        map_pub = n.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid_map", 1);
        
        ROS_INFO("Occupancy Grid Map initialized with map size: %dx%d, resolution: %.2f", 
                 width, height, resolution);
        ROS_INFO("Occupancy thresholds: p_occ=%.2f, p_free=%.2f", p_occ, p_free);
        ROS_INFO("Laser offset: x=%.2f, y=%.2f, yaw=%.2f", laser_offset_x, laser_offset_y, laser_offset_yaw);
    }
    
    /**
     * lidar_callback() uses the current LiDAR scan and Wheel Odometry data to update and publish the Grid Occupancy map
     * Implements the Occupancy Grid Mapping Algorithm from Table 1 in Lab06 instructions
     */
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& data) {
        // Check if we have odometry data
        if (!has_odom) {
            ROS_WARN_THROTTLE(1.0, "Waiting for odometry data...");
            return;
        }
        
        // For each cell in the occupancy grid
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                // Calculate cell center coordinates in odom frame
                double cell_x = origin_x + (col + 0.5) * resolution;
                double cell_y = origin_y + (row + 0.5) * resolution;
                
                // Step 1: Find the closest LiDAR ray to the line connecting cell center to laser origin
                double cell_angle = calculateCellAngle(cell_x, cell_y);
                double cell_range = calculateCellRange(cell_x, cell_y);
                
                // Find the closest beam index
                int beam_idx = -1;
                double min_angle_diff = std::numeric_limits<double>::max();
                
                for (size_t i = 0; i < data->ranges.size(); ++i) {
                    double beam_angle = data->angle_min + i * data->angle_increment;
                    double angle_diff = std::abs(cell_angle - beam_angle);
                    
                    // Wrap angle difference to [-pi, pi]
                    if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
                    
                    if (angle_diff < min_angle_diff) {
                        min_angle_diff = angle_diff;
                        beam_idx = i;
                    }
                }
                
                // Skip if no valid beam found
                if (beam_idx < 0 || beam_idx >= static_cast<int>(data->ranges.size())) continue;
                
                // Get the range measurement from the closest beam
                float beam_range = data->ranges[beam_idx];
                
                // Skip invalid readings
                if (beam_range < data->range_min || beam_range > data->range_max) continue;                



                // Step 2: Inverse Sensor Model - Determine p(m_ij | z_k, x_k)
                double l_inverse;  // Log-odds from inverse sensor model
                
                // Transform beam endpoint to odom frame to check if it falls inside this cell
                double beam_end_x = odom_x + beam_range * std::cos(odom_yaw + cell_angle);
                double beam_end_y = odom_y + beam_range * std::sin(odom_yaw + cell_angle);
                auto end_cell = worldToGrid(beam_end_x, beam_end_y);
                
                bool beam_ends_in_cell = (end_cell.first == row && end_cell.second == col);
                
                // Apply inverse sensor model according to equation (17) and Table 1
                if (beam_ends_in_cell) {
                    // OCCUPIED: LiDAR return falls inside the cell
                    l_inverse = l_occ;
                } else if (beam_range > cell_range) {
                    // FREE: The LiDAR return range is greater than the cell range
                    // (cell is between robot and obstacle)
                    l_inverse = l_free;
                } else {
                    // UNKNOWN: The LiDAR return range is less than the cell range
                    // (obstacle is in front of this cell)
                    l_inverse = l_unknown;
                }
                

                
                // Step 3: Update the cell occupancy using recursive log-odds update (equation 21)
                // l(m_ij | z_0:k, x_0:k) = l(m_ij | z_k, x_k) + l(m_ij | z_0:k-1, x_0:k-1) - l(m_ij)
                int idx = gridToIndex(row, col);
                double new_log_odds = l_inverse + log_odds[idx] - l_prior;
                
                // Clamp log-odds to prevent numerical issues
                if (new_log_odds > 10.0) new_log_odds = 10.0;
                if (new_log_odds < -10.0) new_log_odds = -10.0;
                
                // Store updated log-odds
                log_odds[idx] = new_log_odds;
                
                // Update occupancy grid message with new value
                map_occ_grid_msg.data[idx] = logOddsToOccupancy(new_log_odds);
            }
        }
        
        // Publish to map topic
        map_occ_grid_msg.header.stamp = ros::Time::now();
        map_pub.publish(map_occ_grid_msg);
        
        ROS_DEBUG("Map updated with scan containing %zu beams", data->ranges.size());
    }
    
    /**
     * odom_callback() retrieves the wheel odometry data from the published odom_msg
     * Stores the vehicle's current pose with respect to the fixed "odom" frame
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        // Extract robot position
        odom_x = odom_msg->pose.pose.position.x;
        odom_y = odom_msg->pose.pose.position.y;
        
        // Extract robot orientation (yaw) from quaternion
        tf2::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w
        );
        odom_yaw = tf2::impl::getYaw(q);
        
        has_odom = true;
        
        ROS_DEBUG("Odometry updated: x=%.2f, y=%.2f, yaw=%.2f", odom_x, odom_y, odom_yaw);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "occupancygridmap");
    OccupancyGridMap ogm;
    ros::Duration(0.1).sleep();
    ros::spin();
    return 0;
}