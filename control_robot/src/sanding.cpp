#include <rclcpp/rclcpp.hpp>

#include <behaviortree_ros2/bt_action_node.hpp>
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include <custom_service/action/plan_robot.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

using PlanRobot = custom_service::action::PlanRobot;
using GoalHandlePlanRobot = rclcpp_action::ClientGoalHandle<PlanRobot>;

using namespace BT;

class PlanRobotAction : public RosActionNode<PlanRobot>
{
public:
  PlanRobotAction(const std::string &name, const NodeConfig &conf, const RosNodeParams &params)
      : RosActionNode<PlanRobot>(name, conf, params) {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<unsigned>("order")});
  }

  bool setGoal(Goal &goal) override
  {
    unsigned order = 0;
    if (!getInput("order", order)) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Missing input port: order");
      return false;
    }
    goal.task_number = order;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending PlanRobot goal with order: %u", order);
    return true;
  }

  NodeStatus onResultReceived(const WrappedResult &wr) override
  {
    // std::stringstream ss;
    // ss << "PlanRobot sequence received: ";
    // for (auto number : wr.result->success)
    // {
    //   ss << number << " ";
    // }
    // std::string hogya;
    // if (wr.result->success == true){
    //     hogya = "done"
    // }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal achieved by arm!");

    if (wr.result->success == true){
        return NodeStatus::SUCCESS;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal achieved by arm!");
    }
    else 
    {
        return NodeStatus::FAILURE;

    }
  }

  NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action request failed with error code: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
  {
    // std::stringstream ss;
    // ss << "Partial PlanRobot sequence received: ";
    // for (auto number : feedback->partial_sequence)
    // {
    //   ss << number << " ";
    // }
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", ss.str().c_str());
    return NodeStatus::RUNNING;
  }
};

// Main function
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("PlanRobot_action_client");
  RosNodeParams params;
  params.nh = node;
  params.default_port_value = "robot_server";

  BehaviorTreeFactory factory;
  factory.registerNodeType<PlanRobotAction>("PlanRobot", params);

  std::string pkg_share_path = ament_index_cpp::get_package_share_directory("control_robot");
  std::string tree_file = pkg_share_path + "/behaviour_trees/seq_task.xml";
  auto tree = factory.createTreeFromFile(tree_file);
  NodeStatus status = NodeStatus::FAILURE;

//   auto lc_odom = std::make_shared<Rotating>("lc_odom", con);


try {
    NodeStatus status = NodeStatus::IDLE;
    while (rclcpp::ok() && status != NodeStatus::SUCCESS) {
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        if (status == NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "Behavior Tree failed!");
            break;
        }
        tree.sleep(std::chrono::milliseconds(200));
    }
} catch (const std::exception &e) {
    RCLCPP_FATAL(node->get_logger(), "CRASH: %s", e.what());
    return 1;
}
}
