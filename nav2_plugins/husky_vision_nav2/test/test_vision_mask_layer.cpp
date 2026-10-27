#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <cmath>
#include "husky_vision_nav2/vision_mask_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

class VisionLayerTest : public ::testing::Test
{
protected:
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<rclcpp::Node> sender;
  std::shared_ptr<husky_vision_nav2::VisionMaskLayer> layer;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
  void SetUp() override
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("vision.mask_topic", "/vision_layer_unit/mask")});
    node = std::make_shared<nav2_util::LifecycleNode>("vision_layer_unit", "", options);
    sender = std::make_shared<rclcpp::Node>("vision_layer_sender");
    layered = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered->resizeMap(40, 40, .1, 0., 0.);
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    layer = std::make_shared<husky_vision_nav2::VisionMaskLayer>();
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer->initialize(layered.get(), "vision", tf.get(), node, group);
    executor->add_node(node->get_node_base_interface()); executor->add_node(sender);
    publisher = sender->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/vision_layer_unit/mask", rclcpp::QoS(1).reliable().transient_local());
  }
  void TearDown() override
  {
    executor->remove_node(node->get_node_base_interface()); executor->remove_node(sender);
    executor.reset();
    publisher.reset(); layer.reset(); tf.reset(); layered.reset(); sender.reset(); node.reset();
  }
  nav_msgs::msg::OccupancyGrid mask()
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = "map"; msg.header.stamp = node->now();
    msg.info.width = msg.info.height = 4; msg.info.resolution = .1;
    msg.info.origin.orientation.w = 1.; msg.data.assign(16, -1); msg.data[2*4+1] = 100;
    return msg;
  }
  void send(const nav_msgs::msg::OccupancyGrid & msg)
  {
    publisher->publish(msg);
    for (int i = 0; i < 30; ++i) {executor->spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  }
  std::array<double,4> bounds()
  {
    std::array<double,4> b{1e6,1e6,-1e6,-1e6};
    layer->updateBounds(0,0,0,&b[0],&b[1],&b[2],&b[3]); return b;
  }
};

TEST_F(VisionLayerTest, AddsLethalCostWithoutClearingStaticOrUnknownCells)
{
  send(mask()); bounds(); ASSERT_TRUE(layer->isCurrent());
  auto * master = layered->getCostmap();
  master->setCost(0,0,nav2_costmap_2d::LETHAL_OBSTACLE);
  master->setCost(3,3,nav2_costmap_2d::NO_INFORMATION);
  layer->updateCosts(*master,0,0,40,40);
  EXPECT_EQ(master->getCost(1,2),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(0,0),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(3,3),nav2_costmap_2d::NO_INFORMATION);
}

TEST_F(VisionLayerTest, MovingMaskUpdatesPreviousBoundsAndDoesNotLeaveGhosts)
{
  auto msg = mask(); send(msg); bounds();
  auto * master = layered->getCostmap(); layer->updateCosts(*master,0,0,40,40);
  msg.header.stamp = node->now(); msg.info.origin.position.x = 1.; send(msg);
  auto b = bounds(); EXPECT_LT(b[0],0); EXPECT_GT(b[2],1.4);
  // Master normally recomposes these bounds from its map and obstacle layers.
  master->resetMap(0,0,40,40); master->setCost(8,8,nav2_costmap_2d::LETHAL_OBSTACLE);
  layer->updateCosts(*master,0,0,40,40);
  EXPECT_EQ(master->getCost(1,2),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(11,2),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(8,8),nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(VisionLayerTest, OldOrInvalidMasksMakeCostmapNonCurrent)
{
  auto msg = mask(); msg.header.stamp = node->now()-rclcpp::Duration::from_seconds(1.);
  send(msg); bounds(); EXPECT_FALSE(layer->isCurrent());
  msg.header.stamp = node->now(); send(msg); bounds(); EXPECT_TRUE(layer->isCurrent());
  msg.data[0] = 0; send(msg); bounds(); EXPECT_FALSE(layer->isCurrent());
  EXPECT_FALSE(layer->isClearable());
}

TEST_F(VisionLayerTest, MaskFrameTransformsIntoGlobalCostmap)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map"; transform.child_frame_id = "odom";
  transform.transform.translation.x = 1.;
  transform.transform.rotation.z = std::sqrt(.5); transform.transform.rotation.w = std::sqrt(.5);
  ASSERT_TRUE(tf->setTransform(transform,"unit",true));
  auto msg = mask(); msg.header.frame_id = "odom"; send(msg); bounds();
  ASSERT_TRUE(layer->isCurrent());
  auto * master = layered->getCostmap(); layer->updateCosts(*master,0,0,40,40);
  EXPECT_EQ(master->getCost(7,1),nav2_costmap_2d::LETHAL_OBSTACLE);
}
