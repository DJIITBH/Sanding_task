#include <behaviortree_ros2/bt_action_node.hpp>
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;
std::atomic_bool switchActive{true};

using namespace BT;

class Decision {
private:
  float fx;

public:
  float decisionX = 29.29;

  std::string write(float f) {
    this->fx = f;
    return "hello";
  }

  float read() { return this->fx; }
};

float data1;
float data2;

//-------------------------------------------------------------------------------------
//------------------------------LASER--------------------------------------------------
//-------------------------------------------------------------------------------------

class ReadingLaser : public BT::AsyncActionNode, public rclcpp::Node {

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
    //   RCLCPP_INFO(this->get_logger(), "#### CALLBACK ####");
    RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
                _msg->ranges[100]);
    data2 = _msg->ranges[100];
    data1 = _msg->ranges[0]; // 0
  }

public:
  ReadingLaser(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config), Node("scanner_node") {
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos,
        [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          topic_callback(msg);
        });
  }

  virtual void halt() override;

  BT::NodeStatus tick() override {

    RCLCPP_INFO(this->get_logger(), "LASER DATA ===>: '%f' '%f'", data1, data2);

    float min = 10;
    float current;
    for (int i = 0; i < 200; i++) {
      current = data1;
      if (current < min) {
        min = current;
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    return data1 > 0.5 ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts() { return {}; }
};

void ReadingLaser::halt() {
  RCLCPP_INFO(this->get_logger(), "HALT IS CALLING ");
}

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
    message.data = "I am robot and print data" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  BT::NodeStatus tick() override {
    std::string msg;
    if (getInput("message", msg)) {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topicX", 10);
      auto message = std_msgs::msg::String();
      message.data = "next print " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing next msg: '%s'",
                  message.data.c_str());
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

  // virtual void halt() override;

  BT::NodeStatus tick() override {
    RCLCPP_INFO(this->get_logger(), "MOVING ROBOT");

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.1;
    message.angular.z = 0.0;
    const geometry_msgs::msg::PoseStamped::SharedPtr msg;
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

//--------------------------------------------------------------------

// Simple tree, used to execute once each action.
static const char *xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
            <PrintValue message="start"/>
           <Fallback>
                <ReadingLaser name="scanner"/> 
                <StopRobot name="stop_robot"/>
            </Fallback>
            <PrintValue message="stop"/>
            
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("sleep_client");

  BehaviorTreeFactory factory;

  factory.registerNodeType<PrintValue>("PrintValue");
  factory.registerNodeType<MoveRobot>("MoveRobot");
  factory.registerNodeType<StopRobot>("StopRobot");
  factory.registerNodeType<ReadingLaser>("ReadingLaser");

  auto tree = factory.createTreeFromText(xml_text);

  NodeStatus status = NodeStatus::IDLE;
  BT::NodeConfiguration con = {};
  auto lc_listener = std::make_shared<ReadingLaser>("lc_listener", con);

  while (true) {
    rclcpp::spin_some(lc_listener);
    status = tree.tickRoot();
    tree.sleep(std::chrono::milliseconds(100));
  }

  return 0;
}
