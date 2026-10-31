#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <set>
#include <sstream>
#include <thread>
#include "husky_vision_nav2/odom_obstacle_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"

class OdomObstacleTest : public ::testing::Test
{
protected:
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<rclcpp::Node> sender;
  std::shared_ptr<husky_vision_nav2::OdomObstacleLayer> layer;
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
  void SetUp() override
  {
    if (!rclcpp::ok()) {int argc=0; rclcpp::init(argc,nullptr);}
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("obstacle.scan_topic","/odom_obstacle_test/scan"),
      rclcpp::Parameter("inflation.inflation_radius",.48)});
    node=std::make_shared<nav2_util::LifecycleNode>("odom_obstacle_test","",options);
    sender=std::make_shared<rclcpp::Node>("odom_obstacle_sender");
    tf=std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(),node->get_node_timers_interface()));
    layered=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,false);
    layered->resizeMap(200,200,.05,-5.,-5.);
    layer=std::make_shared<husky_vision_nav2::OdomObstacleLayer>();
    inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();
    auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer->initialize(layered.get(),"obstacle",tf.get(),node,group);
    layered->addPlugin(layer);
    inflation->initialize(layered.get(),"inflation",tf.get(),node,group);
    layered->addPlugin(inflation);
    executor->add_node(node->get_node_base_interface()); executor->add_node(sender);
    publisher=sender->create_publisher<sensor_msgs::msg::LaserScan>("/odom_obstacle_test/scan",rclcpp::SensorDataQoS());
    transform("odom","laser",0,0);
    transform("map","odom",0,0);
    // Let DDS discover both endpoints before the first sensor publication.
    spin(100);
  }
  void TearDown() override
  {
    executor->remove_node(node->get_node_base_interface()); executor->remove_node(sender);
    executor.reset();
    publisher.reset(); inflation.reset(); layer.reset(); layered.reset(); tf.reset(); sender.reset(); node.reset();
  }
  void spin(int milliseconds)
  {
    auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now()<end) {
      executor->spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  void transform(const std::string & parent,const std::string & child,double x,double y)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id=parent;t.child_frame_id=child;
    t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.w=1.;
    ASSERT_TRUE(tf->setTransform(t,"test",true));
  }
  sensor_msgs::msg::LaserScan scan(float range=3.)
  {
    sensor_msgs::msg::LaserScan s;
    s.header.frame_id="laser";s.header.stamp=node->now();
    s.angle_min=0.;s.angle_increment=.01;s.angle_max=s.angle_min+200*s.angle_increment;
    s.range_min=.1;s.range_max=100.;s.ranges.assign(201,INFINITY);
    s.ranges[0]=range;return s;
  }
  void send(sensor_msgs::msg::LaserScan s)
  {publisher->publish(s);spin(25);layered->updateMap(0,0,0);}
  unsigned char cost(double x,double y)
  {
    unsigned int mx,my;
    EXPECT_TRUE(layered->getCostmap()->worldToMap(x,y,mx,my));
    return layered->getCostmap()->getCost(mx,my);
  }
  void saveGrid(const std::string & name,nav2_costmap_2d::Costmap2D * map)
  {
    const char * directory=std::getenv("HUSKY_OBSTACLE_REPORT_DIR");
    if (!directory) {return;}
    std::filesystem::create_directories(directory);
    std::ofstream file(std::filesystem::path(directory)/name);
    for (unsigned int y=0;y<map->getSizeInCellsY();++y) {
      for (unsigned int x=0;x<map->getSizeInCellsX();++x) {file<<int(map->getCost(x,y))<<' ';}
      file<<'\n';
    }
  }
};

TEST(OdomObstaclePlugin, ExportedFactoryIsLoadable)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");
  auto instance=loader.createSharedInstance("husky_vision_nav2::OdomObstacleLayer");
  ASSERT_TRUE(instance);
}

TEST_F(OdomObstacleTest, FirstSingleReturnIsAnObstacleWithoutDebounce)
{
  send(scan());ASSERT_TRUE(layer->isCurrent());
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(OdomObstacleTest, MapCorrectionMovesLiveHitAndInflationWithoutLeavingOldHalo)
{
  send(scan());ASSERT_TRUE(layer->isCurrent());
  EXPECT_GT(cost(3.,.1),nav2_costmap_2d::FREE_SPACE);
  transform("map","odom",0,1.);
  send(scan());ASSERT_TRUE(layer->isCurrent());
  EXPECT_EQ(cost(3.,1.),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(cost(3.,1.1),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(3.,.1),nav2_costmap_2d::FREE_SPACE);
}

TEST_F(OdomObstacleTest, CorrectionReprojectsPreviousHitsEvenBeforeAnotherScanArrives)
{
  send(scan());transform("map","odom",0,1.);
  layered->updateMap(0,0,0);
  ASSERT_TRUE(layer->isCurrent());
  EXPECT_EQ(cost(3.,1.),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::FREE_SPACE);
}

TEST_F(OdomObstacleTest, AcceptsAMCLFutureDatedExpiryButRejectsBeyondItsTolerance)
{
  send(scan());
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id="map";t.child_frame_id="odom";
  t.header.stamp=node->now()+rclcpp::Duration::from_seconds(.4);
  t.transform.translation.y=1.;t.transform.rotation.w=1.;
  ASSERT_TRUE(tf->setTransform(t,"amcl_test",false));
  layered->updateMap(0,0,0);ASSERT_TRUE(layer->isCurrent());
  EXPECT_EQ(cost(3.,1.),nav2_costmap_2d::LETHAL_OBSTACLE);
  t.header.stamp=node->now()+rclcpp::Duration::from_seconds(2.);
  ASSERT_TRUE(tf->setTransform(t,"amcl_test",false));
  layered->updateMap(0,0,0);EXPECT_FALSE(layer->isCurrent());
}

TEST_F(OdomObstacleTest, OnlyALaterFiniteRayClearsOldHit)
{
  send(scan(2.));send(scan(3.));
  EXPECT_EQ(cost(2.,0),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::LETHAL_OBSTACLE);
  auto empty=scan();empty.ranges.assign(201,INFINITY);send(empty);
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(OdomObstacleTest, CloserObstacleDoesNotClearAnOccludedOlderOne)
{
  send(scan(3.));send(scan(2.));
  EXPECT_EQ(cost(2.,0),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(OdomObstacleTest, StaleInputKeepsObstaclesButMakesCostmapNoncurrent)
{
  send(scan());spin(550);layered->updateMap(0,0,0);
  EXPECT_FALSE(layer->isCurrent());
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(layer->isClearable());
}

TEST_F(OdomObstacleTest, MalformedInputCannotClearExistingObstacles)
{
  send(scan());auto bad=scan(4.2);bad.angle_max+=.02;send(bad);
  EXPECT_FALSE(layer->isCurrent());
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(OdomObstacleTest, DoesNotOverwriteStaticLethalOrUnknownCells)
{
  send(scan());auto * master=layered->getCostmap();
  master->setCost(1,1,nav2_costmap_2d::LETHAL_OBSTACLE);
  master->setCost(2,2,nav2_costmap_2d::NO_INFORMATION);
  layer->updateCosts(*master,0,0,200,200);
  EXPECT_EQ(master->getCost(1,1),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(2,2),nav2_costmap_2d::NO_INFORMATION);
}

TEST_F(OdomObstacleTest, ConfiguredFootprintClearingDoesNotEraseAnOutsideObstacle)
{
  std::vector<geometry_msgs::msg::Point> hull(4);
  hull[0].x=.3;hull[0].y=.25;hull[1].x=.3;hull[1].y=-.25;
  hull[2].x=-.3;hull[2].y=-.25;hull[3].x=-.3;hull[3].y=.25;
  layered->setFootprint(hull);
  send(scan(.2));EXPECT_NE(cost(.2,0),nav2_costmap_2d::LETHAL_OBSTACLE);
  send(scan(.4));EXPECT_EQ(cost(.4,0),nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(OdomObstacleTest, ReproducesOldMapFrameGhostInStockNav2ButNotPatchedLayer)
{
  // Both layers receive the SAME live data and TF. Correct localization by 1m
  // perpendicular to the ray. New rays cannot clear the stock layer's old hit.
  node->declare_parameter("stock.observation_sources",std::string("scan"));
  node->declare_parameter("track_unknown_space",false);
  node->declare_parameter("transform_tolerance",.3);
  node->declare_parameter("stock.scan.topic",std::string("/odom_obstacle_test/scan"));
  node->declare_parameter("stock.scan.data_type",std::string("LaserScan"));
  node->declare_parameter("stock.scan.marking",true);
  node->declare_parameter("stock.scan.clearing",true);
  node->declare_parameter("stock.scan.inf_is_valid",false);
  node->declare_parameter("stock.scan.obstacle_max_range",4.);
  node->declare_parameter("stock.scan.raytrace_max_range",4.5);
  node->declare_parameter("stock.scan.obstacle_min_range",.1);
  node->declare_parameter("stock.scan.raytrace_min_range",.1);
  node->declare_parameter("stock.scan.max_obstacle_height",3.);
  node->declare_parameter("stock_inflation.inflation_radius",.48);
  auto stock_map=std::make_unique<nav2_costmap_2d::LayeredCostmap>("map",false,false);
  stock_map->resizeMap(200,200,.05,-5.,-5.);
  auto stock=std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  stock->initialize(stock_map.get(),"stock",tf.get(),node,group);
  stock->activate();
  stock_map->addPlugin(stock);
  auto stock_inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();
  stock_inflation->initialize(stock_map.get(),"stock_inflation",tf.get(),node,group);
  stock_map->addPlugin(stock_inflation);spin(100);
  send(scan());stock_map->updateMap(0,0,0);
  saveGrid("stock-initial.txt",stock_map->getCostmap());
  unsigned int x,y;stock_map->getCostmap()->worldToMap(3.,0,x,y);
  ASSERT_EQ(stock_map->getCostmap()->getCost(x,y),nav2_costmap_2d::LETHAL_OBSTACLE);
  transform("map","odom",0,1.);
  send(scan());stock_map->updateMap(0,0,0);
  EXPECT_EQ(stock_map->getCostmap()->getCost(x,y),nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(cost(3.,0),nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(3.,1.),nav2_costmap_2d::LETHAL_OBSTACLE);
  saveGrid("stock-map-frame.txt",stock_map->getCostmap());
  saveGrid("patched-odom-frame.txt",layered->getCostmap());
}

TEST_F(OdomObstacleTest, RecordedBagGeometryReprojectsWithoutDuplicatingOccupiedCells)
{
  // Optional private-bag fixture: original ranges and scan geometry, with a
  // synthetic clock/TF for native tests. This is NOT moving-robot validation.
  const char * fixture=std::getenv("HUSKY_SCAN_FIXTURE");
  if (!fixture) {GTEST_SKIP()<<"No recorded scan fixture supplied";}
  std::ifstream input(fixture);ASSERT_TRUE(input.good());
  transform("odom","laser",-.2317,0);
  int count=0;std::string line;
  while (std::getline(input,line)) {
    std::istringstream values(line);
    auto s=scan();s.ranges.clear();std::string token;
    values>>s.angle_min>>s.angle_increment>>s.range_min>>s.range_max;
    while (values>>token) {s.ranges.push_back(std::stof(token));}
    ASSERT_GE(s.ranges.size(),180u);
    s.angle_max=s.angle_min+(s.ranges.size()-1)*s.angle_increment;
    send(s);ASSERT_TRUE(layer->isCurrent());++count;
  }
  ASSERT_EQ(count,34);
  auto * master=layered->getCostmap();
  std::set<std::pair<int,int>> expected;
  for (unsigned int y=0;y<200;++y) {
    for (unsigned int x=0;x<200;++x) {
      if (master->getCost(x,y)==nav2_costmap_2d::LETHAL_OBSTACLE && y+40<200) {
        expected.insert({x,y+40});
      }
    }
  }
  ASSERT_GT(expected.size(),0u);
  transform("map","odom",0,2.);layered->updateMap(0,0,0);
  ASSERT_TRUE(layer->isCurrent());
  std::set<std::pair<int,int>> actual;
  for (unsigned int y=0;y<200;++y) {
    for (unsigned int x=0;x<200;++x) {
      if (master->getCost(x,y)==nav2_costmap_2d::LETHAL_OBSTACLE) {actual.insert({x,y});}
    }
  }
  EXPECT_EQ(actual,expected);
}
