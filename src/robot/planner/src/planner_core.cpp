#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : logger_(logger),
    goal_tolerance_(0.5),    // Within 50cm counts as reached
    cost_threshold_(50)      // Cells with cost > 50 are obstacles
{
    RCLCPP_INFO(logger_, "PlannerCore initialized");
}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Point& goal_point)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "sim_world";  // Explicitly set this to sim_world
    
    // Convert start and goal to grid cells
    CellIndex start = poseToCell(start_pose, map);
    CellIndex goal = pointToCell(goal_point, map);
    
    // Check if start and goal are valid
    if (!isValid(start, map) || !isValid(goal, map)) {
        RCLCPP_ERROR(logger_, "Start or goal is invalid!");
        return path;  // Return empty path
    }
    
    RCLCPP_INFO(logger_, "Planning from (%d, %d) to (%d, %d)", 
                start.x, start.y, goal.x, goal.y);
    
    // A* algorithm
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;  // Cost from start
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;  // Parent tracking
    
    // Initialize
    g_score[start] = 0.0;
    open_set.push(AStarNode(start, heuristic(start, goal)));
    
    while (!open_set.empty()) {
        // Get cell with lowest f_score
        AStarNode current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (current.index == goal) {
            RCLCPP_INFO(logger_, "Path found!");
            
            // Reconstruct path
            std::vector<CellIndex> cell_path = reconstructPath(start, goal, came_from);
            
            // Convert to ROS Path message
            for (const auto& cell : cell_path) {
                path.poses.push_back(cellToPose(cell, map));
            }
            
            return path;
        }
        
        // Explore neighbors
        std::vector<CellIndex> neighbors = getNeighbors(current.index);
        
        for (const auto& neighbor : neighbors) {
            // Skip invalid cells
            if (!isValid(neighbor, map)) {
                continue;
            }
            
            // Calculate tentative g_score
            double tentative_g = g_score[current.index] + 1.0;  // Assume cost = 1 per cell
            
            // If this path to neighbor is better than previous
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                // Update path
                came_from[neighbor] = current.index;
                g_score[neighbor] = tentative_g;
                double f_score = tentative_g + heuristic(neighbor, goal);
                
                open_set.push(AStarNode(neighbor, f_score));
            }
        }
    }
    
    RCLCPP_WARN(logger_, "No path found!");
    return path;  // Return empty path
}

CellIndex PlannerCore::poseToCell(
    const geometry_msgs::msg::Pose& pose,
    const nav_msgs::msg::OccupancyGrid& map)
{
    int x = static_cast<int>((pose.position.x - map.info.origin.position.x) / map.info.resolution);
    int y = static_cast<int>((pose.position.y - map.info.origin.position.y) / map.info.resolution);
    return CellIndex(x, y);
}

CellIndex PlannerCore::pointToCell(
    const geometry_msgs::msg::Point& point,
    const nav_msgs::msg::OccupancyGrid& map)
{
    int x = static_cast<int>((point.x - map.info.origin.position.x) / map.info.resolution);
    int y = static_cast<int>((point.y - map.info.origin.position.y) / map.info.resolution);
    return CellIndex(x, y);
}

geometry_msgs::msg::PoseStamped PlannerCore::cellToPose(
    const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid& map)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "sim_world";  // Explicitly set frame
    
    // Convert cell to world coordinates (center of cell)
    pose.pose.position.x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
    pose.pose.position.y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
    pose.pose.position.z = 0.0;
    
    pose.pose.orientation.w = 1.0;  // No rotation
    
    return pose;
}

bool PlannerCore::isValid(
    const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid& map)
{
    // Check bounds
    if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
        cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
        return false;
    }
    
    // Check if cell is obstacle
    int index = cell.y * map.info.width + cell.x;
    int8_t cost = map.data[index];
    
    // Unknown cells (-1) are considered valid (optimistic planning)
    // Cells with high cost are obstacles
    if (cost > cost_threshold_) {
        return false;
    }
    
    return true;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b)
{
    // Euclidean distance
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell)
{
    std::vector<CellIndex> neighbors;
    
    // 8-connected grid (including diagonals)
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;  // Skip self
            
            neighbors.push_back(CellIndex(cell.x + dx, cell.y + dy));
        }
    }
    
    return neighbors;
}

std::vector<CellIndex> PlannerCore::reconstructPath(
    const CellIndex& start,
    const CellIndex& goal,
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from)
{
    std::vector<CellIndex> path;
    CellIndex current = goal;
    
    // Trace back from goal to start
    while (current != start) {
        path.push_back(current);
        current = came_from.at(current);
    }
    path.push_back(start);
    
    // Reverse to get start -> goal
    std::reverse(path.begin(), path.end());
    
    return path;
}

}