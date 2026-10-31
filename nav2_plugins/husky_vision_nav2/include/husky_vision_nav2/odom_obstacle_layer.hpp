#pragma once
#include <array>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include "nav2_costmap_2d/layer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace husky_vision_nav2
{
// Keep measured hits in continuous odometry, not in AMCL's correcting map
// frame. Remove a hit only when a later finite measured ray clears its cell.
class OdomObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  bool isClearable() override {return false;}
private:
  using Cell = std::pair<int, int>;
  using Point = std::array<double, 2>;
  void receive(sensor_msgs::msg::LaserScan::ConstSharedPtr);
  bool integrate(const sensor_msgs::msg::LaserScan &);
  Cell cell(double, double) const;
  void clearRay(double, double, double, double);
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr pending_;
  std::map<Cell, Point> hits_;
  std::vector<Point> projected_;
  std::string odom_;
  double resolution_{.05}, timeout_{.5}, mark_range_{4.}, clear_range_{4.5};
  double map_transform_tolerance_{.5};
  double min_range_{.1};
  int64_t last_stamp_{0};
  bool input_valid_{false}, have_bounds_{false}, capacity_fault_{false};
  std::array<double, 4> previous_bounds_{};
};
}
