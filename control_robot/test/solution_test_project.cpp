#include <behaviortree_ros2/bt_action_node.hpp>
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include <math.h>

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;
std::atomic_bool switchActive{true};

using namespace BT;

float data1;
float data2;
float robotAngle;
float position_x;
float position_y;

//-------------------------------------------------------------------------------------
//------------------------------LASER--------------------------------------------------
//-------------------------------------------------------------------------------------

class ReadingLaser : public BT::SyncActionNode, public rclcpp::Node {

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {

    data2 = _msg->ranges[100];
    data1 = _msg->ranges[0];
  }

public:
  ReadingLaser(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), Node("scanner_node") {
    RCLCPP_INFO(this->get_logger(), "This is a CONSTRUCTOR");
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos,
        [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          topic_callback(msg);
        });
  }

  BT::NodeStatus tick() override {

    auto res = getInput<float>("input");
    if (!res) {
      throw RuntimeError("error reading port [input]:", res.error());
    }
    float angleConstraint = res.value();

    RCLCPP_INFO(this->get_logger(), "This is a tick()-------");
    RCLCPP_INFO(this->get_logger(), "NODE : LASER DATA ===>: '%f' '%f'", data1,
                data2);
    // 200 readings, from right to left, from -57 to 57 degress
    // calculate new velocity cmd
    float min = 10;
    float current;
    for (int i = 0; i < 200; i++) {
      current = data1;
      if (current < min) {
        min = current;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return (data1 > 2 && robotAngle > angleConstraint) ? NodeStatus::SUCCESS
                                                       : NodeStatus::FAILURE;
  }

  static PortsList providedPorts() {

    const char *description = "Simply print the target on console...";
    return {InputPort<float>("input", description)};
  }
};

//-------------------------------------------------------------------------------------
//--------------------------MOVE_ROBOT-------------------------------------------------
//-------------------------------------------------------------------------------------

class MoveRobot : public BT::SyncActionNode, public rclcpp::Node {

public:
  MoveRobot(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), Node("rotating_node") {

    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", sensor_qos, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odometry_callback(msg);
        });
  }

  NodeStatus tick() override {

    auto res = getInput<float>("input");
    if (!res) {
      throw RuntimeError("error reading port [input]:", res.error());
    }
    float x_or_y = res.value();

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto message = geometry_msgs::msg::Twist();

    RCLCPP_INFO(this->get_logger(), "robot position: '%f' '%f'", position_x,
                position_y);

    if (x_or_y == 1) {
      message.linear.x = 0.5;
    }
    if (x_or_y == 2) {
      message.linear.y = 0.5;
    }

    const geometry_msgs::msg::PoseStamped::SharedPtr msg;

    publisher_->publish(message);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return position_x > 3.0 || position_y > 3.0 ? NodeStatus::SUCCESS
                                                : NodeStatus::FAILURE;
  }

  static PortsList providedPorts() {

    const char *description = "Simply print the target on console...";
    return {InputPort<float>("input", description)};
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr _msg) {

    position_x = _msg->pose.pose.position.x;
    position_y = _msg->pose.pose.position.y;
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "yaw: '%f'", robotAngle);
    auto message = geometry_msgs::msg::Twist();

    const geometry_msgs::msg::PoseStamped::SharedPtr msg;

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

//-------------------------------------------------------------------------------------
//---------------------ROTATING_ROBOT--------------------------------------------------
//-------------------------------------------------------------------------------------

class Rotating : public BT::SyncActionNode, public rclcpp::Node {

public:
  Rotating(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), Node("rotating_node") {

    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", sensor_qos, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odometry_callback(msg);
        });
  }

  NodeStatus tick() override {
    auto res = getInput<float>("input");
    if (!res) {
      throw RuntimeError("error reading port [input]:", res.error());
    }
    float angle = res.value();

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "yaw: '%f'", robotAngle);
    RCLCPP_INFO(this->get_logger(), "ROTATE NODE: LASER ===>: '%f' '%f'", data1,
                data2);
    auto message = geometry_msgs::msg::Twist();

    message.angular.z = 0.4;

    const geometry_msgs::msg::PoseStamped::SharedPtr msg;

    publisher_->publish(message);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts() {

    const char *description = "Simply print the target on console...";
    return {InputPort<float>("input", description)};
  }

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr _msg) {

    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = _msg->pose.pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double roll{}, pitch{}, yaw{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(roll, pitch, yaw);

    float yaw_check = yaw * 180 / M_PI;

    if (yaw_check < 0) {
      yaw_check = 360.0 - std::abs(yaw_check);
    }

    robotAngle = yaw_check;

    RCLCPP_INFO(this->get_logger(), "position: '%f' '%f'",
                _msg->pose.pose.position.x, _msg->pose.pose.position.y);
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "yaw: '%f'", robotAngle);
    auto message = geometry_msgs::msg::Twist();
    if (robotAngle < 30.0) {
      message.angular.z = 0.3;
    }

    if (robotAngle > 30.0) {
      message.angular.z = 0.0;
    }

    const geometry_msgs::msg::PoseStamped::SharedPtr msg;

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

//------------------------------------------------------------------------------------------------------------

// Simple tree, used to execute once each action.
static const char *xml_text = R"(
 <root >
     <BehaviorTree>
     <Sequence>
      <SetBlackboard   output_key="Interface" value="360" />
      <SetBlackboard   output_key="Constraint1" value="180" />
      <SetBlackboard   output_key="Constraint2" value="270" />
      <SetBlackboard   output_key="Direction1" value="1" />
      <SetBlackboard   output_key="Direction2" value="2" />
        <ReactiveSequence> 
           <Rotating     input="{Interface}" /> 
           <ReadingLaser input="{Constraint1}"/>
        </ReactiveSequence> 
        <MoveRobot input="{Direction1}"/>
        <ReactiveSequence> 
           <Rotating     input="{Interface}" /> 
           <ReadingLaser input="{Constraint2}"/>
        </ReactiveSequence> 
        <MoveRobot input="{Direction2}"/>
    </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("sleep_client");

  BehaviorTreeFactory factory;

  factory.registerNodeType<MoveRobot>("MoveRobot");
  factory.registerNodeType<ReadingLaser>("ReadingLaser");
  factory.registerNodeType<Rotating>("Rotating");

  auto tree = factory.createTreeFromText(xml_text);

  NodeStatus status = NodeStatus::FAILURE;
  BT::NodeConfiguration con = {};
  // start laser
  auto lc_listener = std::make_shared<ReadingLaser>("lc_listener", con);
  auto lc_odom = std::make_shared<Rotating>("lc_odom", con);

  FileLogger logger_file(tree, "bt_trace_project.fbl");
  while (status == BT::NodeStatus::FAILURE) {
    rclcpp::spin_some(lc_odom);
    rclcpp::spin_some(lc_listener);
    status = tree.tickRoot();
    tree.sleep(std::chrono::milliseconds(200));
  }

  return 0;
}
