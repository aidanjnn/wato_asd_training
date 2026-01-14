#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger),
    resolution_(0.05),       // Match costmap resolution
    grid_width_(400),        // 20m x 20m map (larger than costmap)
    grid_height_(400),
    update_distance_(1.5),   // Update every 1.5m
    first_update_(true)
{
    initializeMap();
    RCLCPP_INFO(logger_, "MapMemoryCore initialized with %dx%d grid", 
                grid_width_, grid_height_);
}

void MapMemoryCore::initializeMap()
{
    // Set frame
    global_map_.header.frame_id = "sim_world";
    
    // Set metadata
    global_map_.info.resolution = resolution_;
    global_map_.info.width = grid_width_;
    global_map_.info.height = grid_height_;
    
    // Set origin (center the map)
    global_map_.info.origin.position.x = -(grid_width_ * resolution_) / 2.0;
    global_map_.info.origin.position.y = -(grid_height_ * resolution_) / 2.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    
    // Initialize all cells to -1 (unknown)
    global_map_.data.resize(grid_width_ * grid_height_, -1);
}

bool MapMemoryCore::shouldUpdateMap(const geometry_msgs::msg::Pose& current_pose)
{
    // Always update on first call
    if (first_update_) {
        last_update_pose_ = current_pose;
        first_update_ = false;
        return true;
    }
    
    // Check if robot has moved enough
    double distance = calculateDistance(current_pose, last_update_pose_);
    
    if (distance >= update_distance_) {
        last_update_pose_ = current_pose;
        return true;
    }
    
    return false;
}

double MapMemoryCore::calculateDistance(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2)
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

void MapMemoryCore::fuseCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap,
    const geometry_msgs::msg::Pose& robot_pose)
{
    // Transform costmap cells to global coordinates
    std::vector<std::pair<int, int>> global_cells;
    std::vector<int8_t> values;
    transformCostmapToGlobal(costmap, robot_pose, global_cells, values);
    
    // Update global map with new data
    for (size_t i = 0; i < global_cells.size(); ++i) {
        int gx = global_cells[i].first;
        int gy = global_cells[i].second;
        int8_t value = values[i];
        
        // Check bounds
        if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_) {
            int index = gy * grid_width_ + gx;
            
            // Update rule: new data overwrites old data
            // (You could make this smarter with probabilistic fusion)
            if (value > 0) {  // Only update if there's actual obstacle data
                global_map_.data[index] = value;
            } else if (global_map_.data[index] == -1) {
                // If cell was unknown and new data says free, mark as free
                global_map_.data[index] = 0;
            }
        }
    }
}

void MapMemoryCore::transformCostmapToGlobal(
    const nav_msgs::msg::OccupancyGrid& costmap,
    const geometry_msgs::msg::Pose& robot_pose,
    std::vector<std::pair<int, int>>& global_cells,
    std::vector<int8_t>& values)
{
    // Get robot position in global frame
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;
    
    // For each cell in the costmap
    for (int cy = 0; cy < static_cast<int>(costmap.info.height); ++cy) {
        for (int cx = 0; cx < static_cast<int>(costmap.info.width); ++cx) {
            int costmap_index = cy * costmap.info.width + cx;
            int8_t value = costmap.data[costmap_index];
            
            // Skip unknown cells
            if (value < 0) continue;
            
            // Convert costmap cell to meters (relative to robot)
            double local_x = (cx - costmap.info.width / 2.0) * costmap.info.resolution;
            double local_y = (cy - costmap.info.height / 2.0) * costmap.info.resolution;
            
            // Transform to global coordinates (simplified - assumes no rotation)
            double global_x = robot_x + local_x;
            double global_y = robot_y + local_y;
            
            // Convert to global map grid coordinates
            int gx = static_cast<int>((global_x - global_map_.info.origin.position.x) 
                                      / global_map_.info.resolution);
            int gy = static_cast<int>((global_y - global_map_.info.origin.position.y) 
                                      / global_map_.info.resolution);
            
            global_cells.push_back({gx, gy});
            values.push_back(value);
        }
    }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap() const
{
    return global_map_;
}

}