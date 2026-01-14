#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger),
    lookahead_distance_(1.0),      // Look 1 meter ahead
    max_linear_velocity_(0.5),     // 0.5 m/s max speed
    max_angular_velocity_(1.0),    // 1.0 rad/s max turn rate
    goal_tolerance_(0.3)           // Within 30cm counts as reached
{
    RCLCPP_INFO(logger_, "ControlCore initialized");
    RCLCPP_INFO(logger_, "  Lookahead: %.2f m", lookahead_distance_);
    RCLCPP_INFO(logger_, "  Max linear vel: %.2f m/s", max_linear_velocity_);
    RCLCPP_INFO(logger_, "  Max angular vel: %.2f rad/s", max_angular_velocity_);
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Pose& current_pose)
{
    geometry_msgs::msg::Twist cmd_vel;
    
    // Check if path is empty
    if (path.poses.empty()) {
        RCLCPP_WARN(logger_, "Path is empty, stopping robot");
        return cmd_vel;  // Zero velocity
    }
    
    // Check if goal is reached
    if (isGoalReached(path, current_pose)) {
        RCLCPP_INFO(logger_, "Goal reached! Stopping.");
        return cmd_vel;  // Zero velocity
    }
    
    // Find lookahead point on path
    geometry_msgs::msg::Point lookahead_point = findLookaheadPoint(path, current_pose);
    
    // Calculate steering angle to lookahead point
    double steering_angle = calculateSteeringAngle(current_pose, lookahead_point);
    
    // Calculate distance to lookahead point
    double distance_to_lookahead = calculateDistance(current_pose.position, lookahead_point);
    
    // Set linear velocity (slow down as we get closer to goal)
    double distance_to_goal = calculateDistance(current_pose.position, path.poses.back().pose.position);
    double linear_velocity = max_linear_velocity_;
    
    // Slow down near goal
    if (distance_to_goal < 1.0) {
        linear_velocity = max_linear_velocity_ * (distance_to_goal / 1.0);
        linear_velocity = std::max(linear_velocity, 0.1);  // Minimum speed
    }
    
    // Set angular velocity based on steering angle
    double angular_velocity = steering_angle;
    
    // Clamp velocities
    linear_velocity = std::clamp(linear_velocity, 0.0, max_linear_velocity_);
    angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);
    
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
    
    return cmd_vel;
}

bool ControlCore::isGoalReached(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Pose& current_pose)
{
    if (path.poses.empty()) {
        return false;
    }
    
    double distance = calculateDistance(
        current_pose.position,
        path.poses.back().pose.position
    );
    
    return distance < goal_tolerance_;
}

double ControlCore::calculateDistance(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Point ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Pose& current_pose)
{
    // Find the point on the path that is closest to lookahead_distance_ ahead
    geometry_msgs::msg::Point lookahead_point = path.poses.back().pose.position;
    
    for (const auto& pose_stamped : path.poses) {
        double distance = calculateDistance(current_pose.position, pose_stamped.pose.position);
        
        if (distance >= lookahead_distance_) {
            lookahead_point = pose_stamped.pose.position;
            break;
        }
    }
    
    return lookahead_point;
}

double ControlCore::calculateSteeringAngle(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::Point& lookahead_point)
{
    // Get current yaw
    double current_yaw = getYawFromQuaternion(current_pose.orientation);
    
    // Calculate angle to lookahead point
    double dx = lookahead_point.x - current_pose.position.x;
    double dy = lookahead_point.y - current_pose.position.y;
    double target_yaw = std::atan2(dy, dx);
    
    // Calculate steering angle (difference between current and target yaw)
    double steering_angle = target_yaw - current_yaw;
    
    // Normalize to [-pi, pi]
    while (steering_angle > M_PI) steering_angle -= 2.0 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2.0 * M_PI;
    
    return steering_angle;
}

double ControlCore::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
{
    // Convert quaternion to yaw angle
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

}