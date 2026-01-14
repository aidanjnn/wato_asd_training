#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() 
  : Node("costmap"), 
    costmap_(robot::CostmapCore(this->get_logger())) 
{
  // Subscribe to lidar
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  
  // Publish costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10);
  
  RCLCPP_INFO(this->get_logger(), "Costmap node initialized");
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Create costmap from laser scan
  auto costmap = costmap_.createCostmap(msg);
  
  // Set timestamp
  costmap.header.stamp = this->get_clock()->now();
  
  // Publish
  costmap_pub_->publish(costmap);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}