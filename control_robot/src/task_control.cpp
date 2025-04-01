#include <behaviortree_ros2/bt_action_node.hpp>
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <custom_service/action/plan_robot.hpp>

#include <memory>
#include <string>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace BT;

class Task1Action : public BT::SyncActionNode , public rclcpp::Node
{
public:
    Task1Action(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
          node_(node),
          client_(rclcpp_action::create_client<custom_service::action::PlanRobot>(
              node_, "plan_robot"))
    {
    }
    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(this->get_logger(), "This is a tick()-------");
        if (!client_->wait_for_action_server())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Action server not available");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_msg = custom_service::action::PlanRobot::Goal();
        goal_msg.task_number = 1;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending goal");

        auto send_goal_options = rclcpp_action::Client<custom_service::action::PlanRobot>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&Task1Action::goal_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&Task1Action::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Task1Action::result_callback, this, _1);

        client_->async_send_goal(goal_msg, send_goal_options);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (flag)
        {
            NodeStatus::SUCCESS;
        }
        else
        {
            NodeStatus::FAILURE;
        }

    }
    
    void goal_callback(const rclcpp_action::ClientGoalHandle<custom_service::action::PlanRobot>::SharedPtr& goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected by the server");
        }
        else
        {
            goal_handle_ = goal_handle;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal was accepted by the server");
        }
    }

    // Callback for feedback from the server
    void feedback_callback(const rclcpp_action::ClientGoalHandle<custom_service::action::PlanRobot>::SharedPtr,
                           const std::shared_ptr<const custom_service::action::PlanRobot::Feedback> feedback)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Feedback received: %d%%", feedback->percentage);
    }

    // Callback for result from the server
    void result_callback(const rclcpp_action::ClientGoalHandle<custom_service::action::PlanRobot>::WrappedResult& result)
    {
        // Check the result code and return the appropriate status
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task 1 completed successfully");
            // Task succeeded, return SUCCESS to Behavior Tree
            flag = true;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task 1 failed or was aborted");
            // Task failed or was aborted, return FAILURE to Behavior Tree
        }
    }

private:
    rclcpp_action::Client<custom_service::action::PlanRobot>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<custom_service::action::PlanRobot>::SharedPtr goal_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr node_;
    bool flag=false;



    // Callback when the goal is accepted by the action server
};


int main(int argc, char **argv ){
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("client_node");

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
    
    //Node registration process
    factory.registerBuilder<Task1Action>("Task1Action",
        [&ros_node](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<Task1Action>(name, config, ros_node);
        });
        
    std::string pkg_share_path = ament_index_cpp::get_package_share_directory("control_robot");
    std::string tree_file = pkg_share_path + "/behaviour_trees/seq_task.xml";
    auto tree = factory.createTreeFromFile(tree_file);

    NodeStatus status = NodeStatus::FAILURE;
    BT::NodeConfiguration con = {};

    auto mynode = std::make_shared<Task1Action>("Task1Action", con);

    while (status == BT::NodeStatus::FAILURE) {
        rclcpp::spin_some(mynode);
        //we check the status of node
        status = tree.tickRoot();
        tree.sleep(std::chrono::milliseconds(200));
      }
    

    return 0;
}
