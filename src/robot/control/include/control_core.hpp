#ifndef CONTROL_CORE_HPP
#define CONTROL_CORE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

namespace robot
{

class ControlCore
{
public:
    ControlCore(const rclcpp::Logger& logger);
    
    // Main control function - returns velocity command
    geometry_msgs::msg::Twist computeVelocity(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::Pose& current_pose);
    
    // Check if goal is reached
    bool isGoalReached(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::Pose& current_pose);
    
    // Setters for parameters
    void setLookaheadDistance(double distance) { lookahead_distance_ = distance; }
    void setMaxLinearVelocity(double velocity) { max_linear_velocity_ = velocity; }
    void setMaxAngularVelocity(double velocity) { max_angular_velocity_ = velocity; }
    void setGoalTolerance(double tolerance) { goal_tolerance_ = tolerance; }
    
private:
    rclcpp::Logger logger_;
    
    // Parameters
    double lookahead_distance_;      // How far ahead to look on path
    double max_linear_velocity_;     // Max forward speed
    double max_angular_velocity_;    // Max turning speed
    double goal_tolerance_;          // Distance to goal that counts as "reached"
    
    // Helper functions
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                            const geometry_msgs::msg::Point& p2);
    
    geometry_msgs::msg::Point findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::Pose& current_pose);
    
    double calculateSteeringAngle(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Point& lookahead_point);
    
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);
};

}

#endif