#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"),
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    has_costmap_(false),
    has_odom_(false)
{
    // Subscribe to costmap
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    
    // Subscribe to odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    
    // Publish global map
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", 10);
    
    // Timer to periodically update map (1 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Map Memory node initialized");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_costmap_ = msg;
    has_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_odom_ = msg;
    has_odom_ = true;
}

void MapMemoryNode::timerCallback()
{
    // Make sure we have both costmap and odometry data
    if (!has_costmap_ || !has_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for costmap and odometry data...");
        return;
    }
    
    // Check if robot has moved enough to update map
    if (map_memory_.shouldUpdateMap(latest_odom_->pose.pose)) {
        RCLCPP_INFO(this->get_logger(), "Updating map at position (%.2f, %.2f)",
                    latest_odom_->pose.pose.position.x,
                    latest_odom_->pose.pose.position.y);
        
        // Fuse costmap into global map
        map_memory_.fuseCostmap(*latest_costmap_, latest_odom_->pose.pose);
    }
    
    // Publish the global map
    auto map = map_memory_.getMap();
    map.header.stamp = this->get_clock()->now();
    map_pub_->publish(map);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}