#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() 
  : Node("planner"), 
    planner_(robot::PlannerCore(this->get_logger())), 
    state_(State::WAITING_FOR_GOAL) 
{
    // Initialize subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            this->mapCallback(msg);
        });
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            this->goalCallback(msg);
        });
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odomCallback(msg);
        });
    
    // Initialize publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    
    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), [this]() { this->timerCallback(); });
    
    RCLCPP_INFO(this->get_logger(), "PlannerNode initialized - listening to /map, waiting for goal");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    map_received_ = true;
    
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    
    RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", 
                goal_.point.x, goal_.point.y);
    
    if (map_received_) {
        planPath();
    }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else if (map_received_) {
            // Replan periodically to handle dynamic obstacles
            planPath();
        }
    }
}

bool PlannerNode::goalReached() {
    if (!goal_received_) {
        return false;
    }
    
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    return distance < GOAL_THRESHOLD;
}

void PlannerNode::planPath() {
    if (!goal_received_ || !map_received_) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }
    
    if (current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Empty map!");
        return;
    }
    
    // Create a path using the planner core
    nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_.point);
    
    if (path.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found to goal!");
        return;
    }
    
    // Publish the path
    path_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.poses.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}