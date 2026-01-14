#include "planner_core.hpp"
#include <algorithm>
#include <limits>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {
    RCLCPP_INFO(logger_, "PlannerCore initialized");
}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Point& goal
) {
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = "sim_world";  // Always use sim_world frame for paths
    
    // Convert world coordinates to grid coordinates
    CellIndex start_cell = worldToGrid(start.position.x, start.position.y, map);
    CellIndex goal_cell = worldToGrid(goal.x, goal.y, map);
    
    // Check if start and goal are valid
    if (!isValidCell(start_cell, map) || !isValidCell(goal_cell, map)) {
        RCLCPP_WARN(logger_, "Invalid start or goal cell");
        return path;
    }
    
    if (isObstacle(start_cell, map) || isObstacle(goal_cell, map)) {
        RCLCPP_WARN(logger_, "Start or goal is an obstacle");
        return path;
    }
    
    // Run A* search
    std::vector<CellIndex> grid_path = aStarSearch(map, start_cell, goal_cell);
    
    if (grid_path.empty()) {
        RCLCPP_WARN(logger_, "No path found");
        return path;
    }
    
    // Convert grid path back to world coordinates
    for (const auto& cell : grid_path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path.header;
        pose_stamped.pose = gridToWorld(cell, map);
        path.poses.push_back(pose_stamped);
    }
    
    RCLCPP_INFO(logger_, "Path planned with %zu waypoints", path.poses.size());
    return path;
}

std::vector<CellIndex> PlannerCore::aStarSearch(
    const nav_msgs::msg::OccupancyGrid& map,
    const CellIndex& start,
    const CellIndex& goal
) {
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, double, CellIndexHash> f_score;
    
    // Initialize scores
    g_score[start] = 0.0;
    f_score[start] = heuristic(start, goal);
    open_set.push(AStarNode(start, f_score[start]));
    
    std::unordered_set<CellIndex, CellIndexHash> closed_set;
    
    while (!open_set.empty()) {
        AStarNode current_node = open_set.top();
        open_set.pop();
        CellIndex current = current_node.index;
        
        // Check if we've reached the goal
        if (current == goal) {
            // Reconstruct path
            std::vector<CellIndex> path;
            CellIndex node = goal;
            while (node != start) {
                path.push_back(node);
                node = came_from[node];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        closed_set.insert(current);
        
        // Explore neighbors
        std::vector<CellIndex> neighbors = getNeighbors(current);
        for (const auto& neighbor : neighbors) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            if (!isValidCell(neighbor, map) || isObstacle(neighbor, map)) {
                continue;
            }
            
            // Calculate tentative g_score
            double tentative_g_score = g_score[current] + 
                ((neighbor.x != current.x && neighbor.y != current.y) ? DIAGONAL_COST : STRAIGHT_COST);
            
            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal);
                
                open_set.push(AStarNode(neighbor, f_score[neighbor]));
            }
        }
    }
    
    return {}; // No path found
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) {
    // Euclidean distance heuristic
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell) {
    std::vector<CellIndex> neighbors;
    
    // 8-connected neighbors
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            neighbors.push_back(CellIndex(cell.x + dx, cell.y + dy));
        }
    }
    
    return neighbors;
}

CellIndex PlannerCore::worldToGrid(
    double x, double y, 
    const nav_msgs::msg::OccupancyGrid& map
) {
    // Use the same coordinate conversion as the working repository
    // Map origin is at the CENTER, not bottom-left
    int grid_x = static_cast<int>(x / map.info.resolution + static_cast<int>(map.info.width / 2));
    int grid_y = static_cast<int>(y / map.info.resolution + static_cast<int>(map.info.height / 2));
    return CellIndex(grid_x, grid_y);
}

geometry_msgs::msg::Pose PlannerCore::gridToWorld(
    const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid& map
) {
    geometry_msgs::msg::Pose pose;
    // Use the same coordinate conversion as the working repository
    // Map origin is at the CENTER, not bottom-left
    pose.position.x = (cell.x - static_cast<int>(map.info.width / 2)) * map.info.resolution;
    pose.position.y = (cell.y - static_cast<int>(map.info.height / 2)) * map.info.resolution;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

bool PlannerCore::isValidCell(
    const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid& map
) {
    return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
           cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

bool PlannerCore::isObstacle(
    const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid& map
) {
    if (!isValidCell(cell, map)) {
        return true;
    }
    
    int index = cell.y * map.info.width + cell.x;
    return map.data[index] > OCCUPIED_THRESHOLD;
}

}