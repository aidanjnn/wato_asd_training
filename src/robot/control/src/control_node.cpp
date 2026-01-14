#include "control_node.hpp"

namespace robot
{

ControlNode::ControlNode()
  : Node("control_node"),
    core_(this->get_logger()),
    has_path_(false),
    has_odom_(false)
{
    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10,
        std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Control loop timer (20 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ControlNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Control node initialized");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    current_path_ = *msg;
    has_path_ = !msg->poses.empty();
    
    if (has_path_) {
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", 
                    msg->poses.size());
    }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_ = msg->pose.pose;
    has_odom_ = true;
}

void ControlNode::timerCallback()
{
    // Check if we have necessary data
    if (!has_path_ || !has_odom_) {
        // Publish zero velocity to stop robot
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }
    
    // Compute velocity command
    geometry_msgs::msg::Twist cmd_vel = core_.computeVelocity(current_path_, current_pose_);
    
    // Publish command
    cmd_vel_pub_->publish(cmd_vel);
    
    // Check if goal reached
    if (core_.isGoalReached(current_path_, current_pose_)) {
        has_path_ = false;  // Clear path
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
    }
}

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot::ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}