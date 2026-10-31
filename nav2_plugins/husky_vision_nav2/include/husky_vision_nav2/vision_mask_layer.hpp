#pragma once
#include <array>
#include <mutex>
#include <string>
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace husky_vision_nav2
{
class VisionMaskLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  bool isClearable() override {return false;}
private:
  void receive(nav_msgs::msg::OccupancyGrid::ConstSharedPtr);
  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr mask_, active_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  double timeout_{0.5}, tx_{0.0}, ty_{0.0}, c_{1.0}, s_{0.0};
  bool have_bounds_{false}, transform_valid_{false};
  std::array<double, 4> previous_bounds_{};
};
}
