#include "husky_vision_nav2/odom_obstacle_layer.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace husky_vision_nav2
{
void OdomObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("costmap node unavailable");}
  declareParameter("scan_topic", rclcpp::ParameterValue("/husky_nav/mapping_scan"));
  declareParameter("odom_frame", rclcpp::ParameterValue("odom"));
  declareParameter("timeout_sec", rclcpp::ParameterValue(.5));
  declareParameter("map_transform_tolerance_sec", rclcpp::ParameterValue(.5));
  declareParameter("obstacle_max_range", rclcpp::ParameterValue(4.));
  declareParameter("raytrace_max_range", rclcpp::ParameterValue(4.5));
  declareParameter("obstacle_min_range", rclcpp::ParameterValue(.1));
  std::string topic;
  node->get_parameter(name_+".scan_topic", topic);
  node->get_parameter(name_+".odom_frame", odom_);
  node->get_parameter(name_+".timeout_sec", timeout_);
  node->get_parameter(name_+".map_transform_tolerance_sec", map_transform_tolerance_);
  node->get_parameter(name_+".obstacle_max_range", mark_range_);
  node->get_parameter(name_+".raytrace_max_range", clear_range_);
  node->get_parameter(name_+".obstacle_min_range", min_range_);
  resolution_ = layered_costmap_->getCostmap()->getResolution();
  if (topic.empty() || odom_.empty() || !std::isfinite(timeout_) || timeout_ <= 0. ||
    !std::isfinite(mark_range_) || !std::isfinite(clear_range_) || !std::isfinite(min_range_) ||
    min_range_ <= 0. || mark_range_ <= min_range_ || clear_range_ < mark_range_ ||
    clear_range_ > 50. || resolution_ < .01 || resolution_ > .5 ||
    !std::isfinite(map_transform_tolerance_) || map_transform_tolerance_ < 0. || map_transform_tolerance_ > 2.)
  {throw std::runtime_error("invalid odom obstacle layer geometry/timeout");}
  enabled_ = true; current_ = false;
  subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(topic,
    rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {receive(msg);});
}

void OdomObstacleLayer::receive(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = msg;
}

OdomObstacleLayer::Cell OdomObstacleLayer::cell(double x, double y) const
{return {int(std::floor(x/resolution_)), int(std::floor(y/resolution_))};}

void OdomObstacleLayer::clearRay(double sx, double sy, double ex, double ey)
{
  auto [x, y] = cell(sx, sy);
  const auto end = cell(ex, ey);
  const int dx = std::abs(end.first-x), dy = std::abs(end.second-y);
  const int ix = x < end.first ? 1 : -1, iy = y < end.second ? 1 : -1;
  int error = dx-dy;
  // Exclude the endpoint cell. Never turn a measured endpoint into free space.
  while (Cell{x,y} != end) {
    hits_.erase({x,y});
    const int twice = 2*error;
    if (twice > -dy) {error -= dy; x += ix;}
    if (twice < dx) {error += dx; y += iy;}
  }
}

bool OdomObstacleLayer::integrate(const sensor_msgs::msg::LaserScan & scan)
{
  auto node = node_.lock();
  if (capacity_fault_ || scan.header.stamp.sec < 0 || scan.header.stamp.nanosec >= 1000000000) {
    input_valid_ = false; return true;
  }
  const rclcpp::Time stamp(scan.header.stamp, node->get_clock()->get_clock_type());
  const double age = (node->now()-stamp).seconds();
  if (stamp.nanoseconds() <= 0 || age < -.05 || age > timeout_ || scan.header.frame_id.empty() ||
    scan.ranges.size() < 180 || scan.ranges.size() > 10000 ||
    !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max) ||
    !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0. ||
    !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
    scan.range_min <= 0. || scan.range_max <= scan.range_min ||
    std::abs(scan.angle_min+(scan.ranges.size()-1)*scan.angle_increment-scan.angle_max) > 1e-4)
  {input_valid_ = false; return true;}
  if (stamp.nanoseconds() <= last_stamp_) {return true;}
  for (const auto range : scan.ranges) {
    if (std::isnan(range) || range < 0. || (std::isfinite(range) &&
      (range < scan.range_min || range > scan.range_max)))
    {input_valid_ = false; return true;}
  }
  tf2::Transform sensor_to_odom;
  try {
    const auto tf = tf_->lookupTransform(odom_, scan.header.frame_id, stamp);
    tf2::fromMsg(tf.transform, sensor_to_odom);
  } catch (const tf2::TransformException &) {input_valid_ = false; return false;}
  const auto origin = sensor_to_odom.getOrigin();
  if (!std::isfinite(origin.x()) || !std::isfinite(origin.y()) ||
    std::abs(origin.x()/resolution_) > 1e8 || std::abs(origin.y()/resolution_) > 1e8 ||
    !std::isfinite(sensor_to_odom.getRotation().length2()) ||
    std::abs(sensor_to_odom.getRotation().length2()-1.) > 1e-3)
  {input_valid_ = false; return true;}
  std::vector<std::array<double,4>> rays;
  std::vector<Point> measured;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < std::max(min_range_, double(scan.range_min))) {
      continue;  // +inf is unknown. It never clears an old hit.
    }
    const double angle = scan.angle_min+i*scan.angle_increment;
    const double length = std::min(range, clear_range_);
    const auto start = sensor_to_odom*tf2::Vector3(min_range_*std::cos(angle), min_range_*std::sin(angle),0);
    const auto end = sensor_to_odom*tf2::Vector3(length*std::cos(angle),length*std::sin(angle),0);
    rays.push_back({start.x(),start.y(),end.x(),end.y()});
    if (range <= mark_range_) {
      const auto hit = sensor_to_odom*tf2::Vector3(range*std::cos(angle),range*std::sin(angle),0);
      measured.push_back({hit.x(),hit.y()});
    }
  }
  // Complete clearing first, then mark ALL current hits. A different ray from
  // the same scan cannot erase a currently observed obstacle.
  for (const auto & ray : rays) {clearRay(ray[0],ray[1],ray[2],ray[3]);}
  for (const auto & hit : measured) {
    const auto key = cell(hit[0],hit[1]);
    if (hits_.size() >= 250000 && hits_.find(key) == hits_.end()) {capacity_fault_ = true; break;}
    hits_[key] = hit;
  }
  last_stamp_ = stamp.nanoseconds(); input_valid_ = true;
  return true;
}

void OdomObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  current_ = false;
  if (have_bounds_) {
    *min_x = std::min(*min_x,previous_bounds_[0]); *min_y = std::min(*min_y,previous_bounds_[1]);
    *max_x = std::max(*max_x,previous_bounds_[2]); *max_y = std::max(*max_y,previous_bounds_[3]);
  }
  if (!node) {return;}
  if (pending_ && integrate(*pending_)) {pending_.reset();}
  const double age = (node->now().nanoseconds()-last_stamp_)/1e9;
  current_ = input_valid_ && last_stamp_ > 0 && age >= -.05 && age <= timeout_ && !capacity_fault_;
  tf2::Transform odom_to_map;
  odom_to_map.setIdentity();
  try {
    if (layered_costmap_->getGlobalFrameID() != odom_) {
      const auto tf = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),odom_,tf2::TimePointZero);
      const rclcpp::Time stamp(tf.header.stamp,node->get_clock()->get_clock_type());
      const double tf_age = (node->now()-stamp).seconds();
      // AMCL intentionally future-dates map->odom by transform_tolerance.
      // Respect that TF validity interval; do not relax sensor timestamp gates.
      if (stamp.nanoseconds() && (tf_age < -map_transform_tolerance_-.05 || tf_age > timeout_)) {
        current_ = false; return;
      }
      tf2::fromMsg(tf.transform,odom_to_map);
    }
  } catch (const tf2::TransformException &) {current_ = false; return;}
  double roll,pitch,yaw;
  if (!std::isfinite(odom_to_map.getRotation().length2()) ||
    std::abs(odom_to_map.getRotation().length2()-1.) > 1e-3)
  {current_ = false; return;}
  tf2::Matrix3x3(odom_to_map.getRotation()).getRPY(roll,pitch,yaw);
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw) ||
    std::abs(roll) > 1e-3 || std::abs(pitch) > 1e-3)
  {current_ = false; return;}
  std::vector<Point> projected;
  std::array<double,4> bounds{INFINITY,INFINITY,-INFINITY,-INFINITY};
  const auto & footprint=getFootprint();
  const double c=std::cos(robot_yaw), s=std::sin(robot_yaw);
  for (auto it=hits_.begin();it!=hits_.end();) {
    const auto p = odom_to_map*tf2::Vector3(it->second[0],it->second[1],0);
    if (!std::isfinite(p.x()) || !std::isfinite(p.y())) {current_ = false; return;}
    // Preserve Nav2's existing footprint clearing using the configured hull,
    // not a guessed exclusion circle around the sensor or robot.
    const double dx=p.x()-robot_x, dy=p.y()-robot_y;
    const double bx=c*dx+s*dy, by=-s*dx+c*dy;
    bool inside=false;
    if (footprint.size()>=3) {
      for (size_t i=0,j=footprint.size()-1;i<footprint.size();j=i++) {
        const auto & a=footprint[i];const auto & b=footprint[j];
        if ((a.y>by)!=(b.y>by) && bx<(b.x-a.x)*(by-a.y)/(b.y-a.y)+a.x) {inside=!inside;}
      }
    }
    if (inside) {it=hits_.erase(it);continue;}
    projected.push_back({p.x(),p.y()});
    bounds[0] = std::min(bounds[0],p.x()-resolution_); bounds[1] = std::min(bounds[1],p.y()-resolution_);
    bounds[2] = std::max(bounds[2],p.x()+resolution_); bounds[3] = std::max(bounds[3],p.y()+resolution_);
    ++it;
  }
  projected_ = std::move(projected);
  have_bounds_ = !projected_.empty();
  if (have_bounds_) {
    previous_bounds_ = bounds;
    *min_x = std::min(*min_x,bounds[0]); *min_y = std::min(*min_y,bounds[1]);
    *max_x = std::max(*max_x,bounds[2]); *max_y = std::max(*max_y,bounds[3]);
  }
}

void OdomObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D & master,int x0,int y0,int x1,int y1)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Only add occupied cells. Normal master recomposition restores previous
  // bounds from StaticLayer, so no mapped obstacle is erased by this layer.
  for (const auto & point : projected_) {
    unsigned int x,y;
    if (master.worldToMap(point[0],point[1],x,y) && int(x)>=x0 && int(x)<x1 && int(y)>=y0 && int(y)<y1) {
      master.setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void OdomObstacleLayer::reset()
{
  // Do not silently erase observed obstacles via clear-costmap recovery calls.
  std::lock_guard<std::mutex> lock(mutex_);
  current_ = false;
}
}
PLUGINLIB_EXPORT_CLASS(husky_vision_nav2::OdomObstacleLayer,nav2_costmap_2d::Layer)
