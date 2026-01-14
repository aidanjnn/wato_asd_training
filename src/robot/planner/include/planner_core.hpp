#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace robot
{

// Structure for 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex (needed for unordered_map)
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure for A* node (cell + cost)
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h (total estimated cost)

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;  // Smaller f_score = higher priority
  }
};

class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    
    // Main A* planning function
    nav_msgs::msg::Path planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Point& goal_point);

private:
    rclcpp::Logger logger_;
    
    // A* parameters
    double goal_tolerance_;  // How close to goal counts as "reached" (0.5m)
    int cost_threshold_;     // Cell cost above this is obstacle (50)
    
    // Helper functions
    CellIndex poseToCell(
        const geometry_msgs::msg::Pose& pose,
        const nav_msgs::msg::OccupancyGrid& map);
    
    CellIndex pointToCell(
        const geometry_msgs::msg::Point& point,
        const nav_msgs::msg::OccupancyGrid& map);
    
    geometry_msgs::msg::PoseStamped cellToPose(
        const CellIndex& cell,
        const nav_msgs::msg::OccupancyGrid& map);
    
    bool isValid(
        const CellIndex& cell,
        const nav_msgs::msg::OccupancyGrid& map);
    
    double heuristic(
        const CellIndex& a,
        const CellIndex& b);
    
    std::vector<CellIndex> getNeighbors(const CellIndex& cell);
    
    std::vector<CellIndex> reconstructPath(
        const CellIndex& start,
        const CellIndex& goal,
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from);
};

}

#endif