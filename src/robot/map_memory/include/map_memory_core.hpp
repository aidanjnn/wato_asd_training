#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace robot
{

class MapMemoryCore {
public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    
    // Initialize the global map
    void initializeMap();
    
    // Check if robot has moved enough to update map
    bool shouldUpdateMap(const geometry_msgs::msg::Pose& current_pose);
    
    // Fuse a costmap into the global map
    void fuseCostmap(
        const nav_msgs::msg::OccupancyGrid& costmap,
        const geometry_msgs::msg::Pose& robot_pose);
    
    // Get the current global map
    nav_msgs::msg::OccupancyGrid getMap() const;

private:
    rclcpp::Logger logger_;
    
    // Global map parameters
    double resolution_;        // Same as costmap (0.05m)
    int grid_width_;          // Larger than costmap to store history
    int grid_height_;         // Larger than costmap
    double update_distance_;  // Distance threshold to trigger update (1.5m)
    
    // Global map data
    nav_msgs::msg::OccupancyGrid global_map_;
    
    // Last position where map was updated
    geometry_msgs::msg::Pose last_update_pose_;
    bool first_update_;
    
    // Helper functions
    double calculateDistance(
        const geometry_msgs::msg::Pose& pose1,
        const geometry_msgs::msg::Pose& pose2);
    
    void transformCostmapToGlobal(
        const nav_msgs::msg::OccupancyGrid& costmap,
        const geometry_msgs::msg::Pose& robot_pose,
        std::vector<std::pair<int, int>>& global_cells,
        std::vector<int8_t>& values);
};

}

#endif