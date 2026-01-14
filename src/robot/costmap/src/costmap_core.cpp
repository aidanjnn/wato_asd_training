#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) 
  : logger_(logger),
    resolution_(0.1),      // 10cm per cell
    grid_width_(100),      // 10m total width
    grid_height_(100),     // 10m total height
    inflation_radius_(0.5) // 50cm inflation
{
  RCLCPP_INFO(logger_, "CostmapCore initialized");
}

nav_msgs::msg::OccupancyGrid CostmapCore::createCostmap(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
  nav_msgs::msg::OccupancyGrid costmap;
  
  // Initialize the costmap
  initializeCostmap(costmap);
  
  // Process each laser scan point
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    
    // Check if range is valid
    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      
      // Check if within grid bounds
      if (x_grid >= 0 && x_grid < grid_width_ && 
          y_grid >= 0 && y_grid < grid_height_) {
        markObstacle(costmap.data, x_grid, y_grid);
      }
    }
  }
  
  // Inflate obstacles
  inflateObstacles(costmap.data);
  
  return costmap;
}

void CostmapCore::initializeCostmap(nav_msgs::msg::OccupancyGrid& costmap)
{
  // Set metadata
  costmap.header.frame_id = "robot/chassis/lidar";
  costmap.info.resolution = resolution_;
  costmap.info.width = grid_width_;
  costmap.info.height = grid_height_;
  
  // Set origin (center of grid at robot position)
  costmap.info.origin.position.x = -(grid_width_ * resolution_) / 2.0;
  costmap.info.origin.position.y = -(grid_height_ * resolution_) / 2.0;
  costmap.info.origin.position.z = 0.0;
  costmap.info.origin.orientation.w = 1.0;
  
  // Initialize all cells to 0 (free space)
  costmap.data.resize(grid_width_ * grid_height_, 0);
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid)
{
  // Convert polar to Cartesian
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  
  // Convert to grid coordinates (centered at robot)
  x_grid = static_cast<int>((x / resolution_) + (grid_width_ / 2));
  y_grid = static_cast<int>((y / resolution_) + (grid_height_ / 2));
}

void CostmapCore::markObstacle(std::vector<int8_t>& grid, int x, int y)
{
  int index = y * grid_width_ + x;
  grid[index] = 100; // Mark as occupied
}

void CostmapCore::inflateObstacles(std::vector<int8_t>& grid)
{
  std::vector<int8_t> temp_grid = grid; // Copy for reading
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
  
  for (int y = 0; y < grid_height_; ++y) {
    for (int x = 0; x < grid_width_; ++x) {
      int index = y * grid_width_ + x;
      
      // If this cell is an obstacle
      if (temp_grid[index] == 100) {
        // Inflate around it
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_) {
              double distance = std::sqrt(dx*dx + dy*dy) * resolution_;
              
              if (distance <= inflation_radius_) {
                int n_index = ny * grid_width_ + nx;
                
                // Calculate cost based on distance
                int cost = static_cast<int>(100 * (1.0 - distance / inflation_radius_));
                
                // Only update if higher cost
                if (cost > grid[n_index]) {
                  grid[n_index] = cost;
                }
              }
            }
          }
        }
      }
    }
  }
}

}