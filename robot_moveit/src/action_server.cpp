#include<rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include<custom_service/action/fibonacci.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include<thread>

using namespace std::placeholders;

namespace robot_moveit
{
class ActionServer : public rclcpp::Node
{
public:
    explicit ActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("action_server", options)
    {
        action_server_ = rclcpp_action::create_server<custom_service::action::Fibonacci>(
            this, "fibonacci", std::bind(&ActionServer::goal_callback, this, _1, _2), 
            std::bind(&ActionServer::cancel_callback, this, _1), 
            std::bind(&ActionServer::accept_callback, this, _1)
        );
        //goal call back is called when action server receives a new goal 
        // then it decides whether to accept it or reject it then accept call back is executed!
        // if it receives a cancellation request then goal is cancelled using cancel callback function
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"starting the action server");

    }
private:
    rclcpp_action::Server<custom_service::action::Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const custom_service::action::Fibonacci::Goal>goal)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), 
            "Received goal request with order %d", 
            goal->order
        );
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        //we are accepting the goal everytime it comes...
    }
    void accept_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::Fibonacci>> goal_handle)
    {
        //in order to avoid blocking of current thread we will run the logic in a different thread
        //not to keep client blocked for too long while waiting for server 
        // to execute and finsh the accepted callback function
        std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executing goal..");
        rclcpp::Rate loop_rate(1);
        // to control execution of a loop continuous execution of a loop will be separated

        const auto goal = goal_handle->get_goal();
        //contains the goal message...

        auto feedback = std::make_shared<custom_service::action::Fibonacci::Feedback>();
        auto& sequence = feedback->partial_sequence;

        sequence.push_back(0);
        sequence.push_back(1);
        //make_shared is intitialize + making of pointer
        auto result = std::make_shared<custom_service::action::Fibonacci::Result>();

        for(int i=1; i<goal->order && rclcpp::ok(); i++)
        {   
            if (goal_handle->is_canceling()){
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal canceled");
                return;
            }
            sequence.push_back(sequence[i]+sequence[i-1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "publish feedback");
            loop_rate.sleep();
            //stops execution of for loop to match frequency defined
        }

        if(rclcpp::ok())
        {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded!");

        }
    }
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "received request to cancel the goal!");
        return rclcpp_action::CancelResponse::ACCEPT;
        //action server accepted goal cancellation.

    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_moveit::ActionServer)
