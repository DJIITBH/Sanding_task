// we need to include header both for BT and ROS2
// there are also includes to the standard C++ libraries
//----------------------------------------------------------------------------------
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

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;
std::atomic_bool switchActive{true};

using namespace BT;
//------------------------------------------------------------------------------------
// IMPORTANT!!
// Auxiliary variables that hold the data from the subscriber
// and later can be delivered to the BT node
// BESIDE subscriber and BT node are defined in the same class!!!
float data1;
float data2;
float robotAngle;
//-------------------------------------------------------------------------------------

// we have 5 robot actions - 5 ROS2 nodes: Laser, Print, Move robot, Stop Robot
// and Rotate Robot the "architecture" of all included ROS nodes is the same
//-------------------------------------------------------------------------------------
//------------------------------LASER--------------------------------------------------
//-------------------------------------------------------------------------------------

// standard ROS2 node with BT: public BT::SyncActionNode
class ReadingLaser : public BT::SyncActionNode, public rclcpp::Node {

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {

    data2 = _msg->ranges[100];
    data1 = _msg->ranges[0];
  }

public:
  // Node constructor

  // See how we define the subscriber. We use lambda function which "provide
  // mechanism"
  //  to collect data and later reuse data in BT node.

  ReadingLaser(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), Node("scanner_node") {
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos,
        [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          topic_callback(msg);
        });
  }
  // As previously.
  // You must override the virtual function tick()
  BT::NodeStatus tick() override {

    RCLCPP_INFO(this->get_logger(), "This is a tick()-------");
    RCLCPP_INFO(this->get_logger(), "NODE : LASER DATA ===>: '%f' '%f'", data1,
                data2);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    return data1 > 3 ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }
  // Each Node can include port IN/OUT for "communication" between BT node
  // and BlackBoard. Dateil will be proivided later
  // Here all is empty

  static BT::PortsList providedPorts() { return {}; }
};

//-------------------------------------------------------------------------------------
//------------------------------PRINT--------------------------------------------------
//-------------------------------------------------------------------------------------

class PrintValue : public BT::SyncActionNode, public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

public:
  PrintValue(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), Node("minimum_publisher") {}

  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "This is a robot" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  BT::NodeStatus tick() override {
    std::string msg;
    if (getInput("message", msg)) {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topicX", 10);
      auto message = std_msgs::msg::String();
      message.data = "Robot again ! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing:'%s'", message.data.c_str());
      publisher_->publish(message);

      return NodeStatus::SUCCESS;
    } else {

      return NodeStatus::SUCCESS;
    }
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }
};

//-------------------------------------------------------------------------------------
//--------------------------MOVE_ROBOT-------------------------------------------------
//-------------------------------------------------------------------------------------

class MoveRobot : public BT::SyncActionNode, public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;

public:
  MoveRobot(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), Node("moving_robot") {}

  BT::NodeStatus tick() override {
    RCLCPP_INFO(this->get_logger(), "MOVING ROBOT");
    RCLCPP_INFO(this->get_logger(), "MOVE AND LASER ===>: '%f' '%f'", data1,
                data2);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto message = geometry_msgs::msg::Twist();

    message.linear.x = 0.1;

    publisher_->publish(message);

    return NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() { return {}; }
};

//-------------------------------------------------------------------------------------
//--------------------------STOP_ROBOT-------------------------------------------------
//-------------------------------------------------------------------------------------

class StopRobot : public BT::SyncActionNode, public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;

public:
  StopRobot(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), Node("stop_robot") {}

  BT::NodeStatus tick() override {
    RCLCPP_INFO(this->get_logger(), "STOPPING");
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;
    const geometry_msgs::msg::PoseStamped::SharedPtr msg;
    publisher_->publish(message);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    return NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts() { return {}; }
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
    printf("Angle positions: [ %.1f]\n", angle);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "yaw: '%f'", robotAngle);
    RCLCPP_INFO(this->get_logger(), "ROTATE NODE: LASER ===>: '%f' '%f'", data1,
                data2);
    auto message = geometry_msgs::msg::Twist();

    if (robotAngle < angle) {
      message.angular.z = 0.3;
    }

    if (robotAngle > angle) {
      message.angular.z = 0.0;
    }

    const geometry_msgs::msg::PoseStamped::SharedPtr msg;

    publisher_->publish(message);

    std::this_thread::sleep_for(std::chrono::milliseconds(1250));
    return robotAngle > angle ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
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

    robotAngle = yaw * 180 / M_PI;

    RCLCPP_INFO(this->get_logger(), "position: '%f' '%f'",
                _msg->pose.pose.position.x, _msg->pose.pose.position.y);
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "yaw: '%f'", robotAngle);
    auto message = geometry_msgs::msg::Twist();
    // message.linear.x = 0.0;
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
//------------------------------------------------------------------------------------------------------------
// //definition of BT which reflects logical connection between robot actions
// Simple tree, used to execute once each action.

static const char *xml_text = R"(
 <root >
     <BehaviorTree>
     <Sequence>
         <SetBlackboard   output_key="Interface" value="45" />
         <Rotating     input="{Interface}" /> 
        <Fallback>
            <ReadingLaser name="scanner"/>
            <Sequence>
              <SetBlackboard   output_key="Interface" value="90" />
              <Rotating     input="{Interface}" />
                <Fallback>
                    <ReadingLaser name="scanner"/>
                    <Sequence>
                        <SetBlackboard   output_key="Interface" value="135" />
                        <Rotating     input="{Interface}" />
                        <ReadingLaser name="scanner"/>
                    </Sequence>
                </Fallback>
            </Sequence>
        </Fallback>    
    </Sequence>
     </BehaviorTree>
 </root>
 )";
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("sleep_client");

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  // Node registration process
  factory.registerNodeType<PrintValue>("PrintValue");
  factory.registerNodeType<MoveRobot>("MoveRobot");
  factory.registerNodeType<StopRobot>("StopRobot");
  factory.registerNodeType<ReadingLaser>("ReadingLaser");
  factory.registerNodeType<Rotating>("Rotating");

  // we incorporated the BT (XML format)
  auto tree = factory.createTreeFromText(xml_text);

  NodeStatus status = NodeStatus::FAILURE;
  BT::NodeConfiguration con = {};
  // start laser
  // definiion of smart pointers
  auto lc_listener = std::make_shared<ReadingLaser>("lc_listener", con);
  auto lc_odom = std::make_shared<Rotating>("lc_odom", con);
  // for logging purposes. Details later
  FileLogger logger_file(tree, "bt_trace_unit1.fbl");
  while (status == BT::NodeStatus::FAILURE) {
    rclcpp::spin_some(lc_odom);
    rclcpp::spin_some(lc_listener);
    // we check the status of node
    status = tree.tickRoot();
    tree.sleep(std::chrono::milliseconds(200));
  }

  return 0;
}
