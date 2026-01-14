#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "control_core.hpp"

namespace robot
{

class ControlNode : public rclcpp::Node
{
public:
    ControlNode();
    
private:
    // Core logic
    ControlCore core_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Timer for control loop
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Current state
    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Pose current_pose_;
    bool has_path_;
    bool has_odom_;
    
    // Callbacks
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
};

}

#endif