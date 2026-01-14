#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);
    
    //this will process the laser scan, create the costmap as well
    nav_msgs::msg::OccupancyGrid createCostmap(
      const sensor_msgs::msg::LaserScan::SharedPtr& scan);

  private:
    rclcpp::Logger logger_;
    
    // Costmap parameters
    double resolution_;        // meters per cell
    int grid_width_;          // number of cells
    int grid_height_;         // number of cells
    double inflation_radius_; // meters to inflate obstacles
    
    // Helper functions
    void initializeCostmap(nav_msgs::msg::OccupancyGrid& costmap);
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(std::vector<int8_t>& grid, int x, int y);
    void inflateObstacles(std::vector<int8_t>& grid);
};

}  

#endif