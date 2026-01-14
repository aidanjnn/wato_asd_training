#include "planner_node.hpp"

PlannerNode::PlannerNode()
  : Node("planner"),
    planner_(robot::PlannerCore(this->get_logger())),
    state_(State::WAITING_FOR_GOAL),
    has_map_(false),
    has_goal_(false),
    has_odom_(false),
    goal_tolerance_(0.5)  // 50cm
{
    // Subscribe to map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    
    // Subscribe to goal
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10,
        std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    
    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    
    // Publish path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    
    // Timer for periodic checks (2 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Planner node initialized");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = msg;
    has_map_ = true;
    
    // If we're waiting for completion and map updated, replan
    if (state_ == State::WAITING_FOR_COMPLETION && has_goal_ && has_odom_) {
        replan();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    current_goal_ = msg;
    has_goal_ = true;
    
    RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
                msg->point.x, msg->point.y);
    
    // Transition to planning state
    state_ = State::PLANNING;
    
    // Plan immediately
    replan();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = msg;
    has_odom_ = true;
}

void PlannerNode::timerCallback()
{
    if (state_ == State::WAITING_FOR_COMPLETION) {
        // Check if goal reached
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
            has_goal_ = false;
        }
    }
    
    // Always publish current path (even if empty)
    if (has_map_) {
        current_path_.header.stamp = this->get_clock()->now();
        path_pub_->publish(current_path_);
    }
}

bool PlannerNode::goalReached()
{
    if (!has_odom_ || !has_goal_) {
        return false;
    }
    
    double dx = current_odom_->pose.pose.position.x - current_goal_->point.x;
    double dy = current_odom_->pose.pose.position.y - current_goal_->point.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    return distance < goal_tolerance_;
}

void PlannerNode::replan()
{
    if (!has_map_ || !has_goal_ || !has_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for map, goal, and odometry...");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Planning path...");
    
    // Plan path using A*
    current_path_ = planner_.planPath(
        *current_map_,
        current_odom_->pose.pose,
        current_goal_->point);
    
    if (current_path_.poses.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find path!");
        state_ = State::WAITING_FOR_GOAL;
    } else {
        RCLCPP_INFO(this->get_logger(), "Path found with %zu waypoints",
                    current_path_.poses.size());
        state_ = State::WAITING_FOR_COMPLETION;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}