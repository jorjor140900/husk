#include "husky_vision_nav2/vision_mask_layer.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <stdexcept>
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace husky_vision_nav2
{
void VisionMaskLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("costmap node unavailable");}
  declareParameter("mask_topic", rclcpp::ParameterValue("/husky_nav/vision_mask"));
  declareParameter("timeout_sec", rclcpp::ParameterValue(0.5));
  std::string topic;
  node->get_parameter(name_ + ".mask_topic", topic);
  node->get_parameter(name_ + ".timeout_sec", timeout_);
  if (topic.empty() || !std::isfinite(timeout_) || timeout_ <= 0.0) {
    throw std::runtime_error("invalid AI mask topic/timeout");
  }
  enabled_ = true;
  current_ = false;
  subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&VisionMaskLayer::receive, this, std::placeholders::_1));
}

void VisionMaskLayer::receive(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & m = msg->info;
  const auto & p = m.origin.position;
  const auto & q = m.origin.orientation;
  bool valid = !msg->header.frame_id.empty() && msg->header.stamp.sec >= 0 &&
    msg->header.stamp.nanosec < 1000000000 &&
    (msg->header.stamp.sec > 0 || msg->header.stamp.nanosec > 0) && m.width > 0 && m.height > 0 &&
    uint64_t(m.width) * m.height <= 1000000 && msg->data.size() == uint64_t(m.width) * m.height &&
    std::isfinite(m.resolution) && m.resolution > 0.0 &&
    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
    std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6 &&
    std::abs(std::abs(q.w) - 1.0) < 1e-6;
  if (valid) {
    valid = std::all_of(msg->data.begin(), msg->data.end(), [](int8_t x) {return x == -1 || x == 100;});
  }
  mask_ = valid ? msg : nullptr;
  if (!valid) {current_ = false;}
}

void VisionMaskLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  active_ = mask_;
  current_ = false;
  transform_valid_ = false;
  // Clear previous overlay bounds through normal layer recomposition, preserving
  // obstacles from StaticLayer / ObstacleLayer. No free cost is written here.
  if (have_bounds_) {
    *min_x = std::min(*min_x, previous_bounds_[0]); *min_y = std::min(*min_y, previous_bounds_[1]);
    *max_x = std::max(*max_x, previous_bounds_[2]); *max_y = std::max(*max_y, previous_bounds_[3]);
  }
  if (!node || !active_) {return;}
  const auto stamp = rclcpp::Time(active_->header.stamp, node->get_clock()->get_clock_type());
  const double age = (node->now() - stamp).seconds();
  current_ = stamp.nanoseconds() > 0 && age >= -0.05 && age <= timeout_;
  tx_ = ty_ = s_ = 0.0; c_ = 1.0;
  try {
    const auto target = layered_costmap_->getGlobalFrameID();
    if (target != active_->header.frame_id) {
      const auto transform = tf_->lookupTransform(target, active_->header.frame_id, tf2::TimePointZero);
      tf2::Quaternion q;
      tf2::fromMsg(transform.transform.rotation, q);
      const auto & t = transform.transform.translation;
      if (!std::isfinite(q.length2()) || std::abs(q.length2() - 1.0) > 1e-3 ||
        !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z))
      {
        current_ = false; return;
      }
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      if (std::abs(roll) > 1e-3 || std::abs(pitch) > 1e-3) {
        current_ = false; return;
      }
      tx_ = transform.transform.translation.x; ty_ = transform.transform.translation.y;
      c_ = std::cos(yaw); s_ = std::sin(yaw);
    }
  } catch (const tf2::TransformException &) {current_ = false; return;}
  transform_valid_ = true;
  const auto & info = active_->info;
  std::array<double, 4> bounds{std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for (int ix = 0; ix < 2; ++ix) {
    for (int iy = 0; iy < 2; ++iy) {
      const double x = info.origin.position.x + ix * info.width * info.resolution;
      const double y = info.origin.position.y + iy * info.height * info.resolution;
      const double wx = c_ * x - s_ * y + tx_, wy = s_ * x + c_ * y + ty_;
      bounds[0] = std::min(bounds[0], wx-info.resolution);
      bounds[1] = std::min(bounds[1], wy-info.resolution);
      bounds[2] = std::max(bounds[2], wx+info.resolution);
      bounds[3] = std::max(bounds[3], wy+info.resolution);
    }
  }
  *min_x = std::min(*min_x, bounds[0]); *min_y = std::min(*min_y, bounds[1]);
  *max_x = std::max(*max_x, bounds[2]); *max_y = std::max(*max_y, bounds[3]);
  previous_bounds_ = bounds;
  have_bounds_ = true;
}

void VisionMaskLayer::updateCosts(nav2_costmap_2d::Costmap2D & master, int x0, int y0, int x1, int y1)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !transform_valid_) {return;}
  const auto & info = active_->info;
  for (int y = std::max(0, y0); y < std::min(y1, int(master.getSizeInCellsY())); ++y) {
    for (int x = std::max(0, x0); x < std::min(x1, int(master.getSizeInCellsX())); ++x) {
      double wx, wy;
      master.mapToWorld(x, y, wx, wy);
      const double dx = wx-tx_, dy = wy-ty_;
      const double mx = std::floor((c_*dx+s_*dy-info.origin.position.x)/info.resolution);
      const double my = std::floor((-s_*dx+c_*dy-info.origin.position.y)/info.resolution);
      if (mx >= 0 && my >= 0 && mx < info.width && my < info.height &&
        active_->data[size_t(my)*info.width+size_t(mx)] == 100)
      {
        master.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void VisionMaskLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  mask_.reset(); active_.reset(); current_ = false; transform_valid_ = false;
}
}
PLUGINLIB_EXPORT_CLASS(husky_vision_nav2::VisionMaskLayer, nav2_costmap_2d::Layer)
