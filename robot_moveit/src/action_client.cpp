#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_service/action/fibonacci.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robot_moveit
{
class ActionClient : public rclcpp::Node
{
public:
    explicit ActionClient(const rclcpp::NodeOptions& options) : Node("action_client", options)
    {
        client_ = rclcpp_action::create_client<custom_service::action::Fibonacci>(this, "fibonacci"); //this, name of action server
        timer_ =create_wall_timer(1s, std::bind(&ActionClient::timer_callback, this));
        cancel_timer_ = this->create_wall_timer(5s, std::bind(&ActionClient::cancel_goal, this));

    }
private:
// create a clieny object and pass comm interface in the template class!
    rclcpp_action::Client<custom_service::action::Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
    rclcpp_action::ClientGoalHandle<custom_service::action::Fibonacci>::SharedPtr goal_handle_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;


    void timer_callback()
    {
        //executed only once and after 1 second of initialization in constructor..
        timer_->cancel();
        //verify action server is running..
        if(!client_->wait_for_action_server())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "action server not available after waiting");
            rclcpp::shutdown();
        }
        auto goal_msg = custom_service::action::Fibonacci::Goal();
        goal_msg.order = 50;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending goal");

        auto send_goal_options = rclcpp_action::Client<custom_service::action::Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&ActionClient::goal_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&ActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&ActionClient::result_callback, this, _1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_callback(const rclcpp_action::ClientGoalHandle<custom_service::action::Fibonacci>::SharedPtr& goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "goal was rejected by server");

        }
        else{
            goal_handle_ = goal_handle;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal was accepted by server");
        }
    }

    void cancel_goal()
    {
    if (!goal_handle_)  // Check if goal exists before trying to cancel
    {
        RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Requesting goal cancellation...");

    auto future_cancel = client_->async_cancel_goal(goal_handle_);

    cancel_timer_->cancel(); // Stop the cancel timer after use

    // rclcpp::executors::SingleThreadedExecutor executor;

    // // Wait for the cancel response
    // if (executor.spin_until_future_complete(future_cancel, std::chrono::seconds(2)) != 
    //     rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to get response from action server.");
    //     return;
    // }

    // // Check the response from cancel request
    // auto cancel_response = future_cancel.get();

    // if (!cancel_response || cancel_response->goals_canceling.empty())
    // {
    //     RCLCPP_WARN(this->get_logger(), "Goal cancellation was rejected.");
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "Goal successfully canceled.");
    // }

}

    
    void feedback_callback(const rclcpp_action::ClientGoalHandle<custom_service::action::Fibonacci>::SharedPtr, 
        const std::shared_ptr<const custom_service::action::Fibonacci::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "next number in sequence received: ";

            for(auto number : feedback->partial_sequence)
            {
                ss << number << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());

        }
    void result_callback(const rclcpp_action::ClientGoalHandle<custom_service::action::Fibonacci>::WrappedResult& result)
        {
            switch(result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "goal was canceled");
                    rclcpp::shutdown();
                    return;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "unknown result code");
                    return;

            }
            std::stringstream ss;
            ss << "Result received: ";

            for(auto number : result.result->sequence)
            {
                ss << number << " ";
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str().c_str());
            rclcpp::shutdown();

        }
};
}

//register the node with rclcpp components

RCLCPP_COMPONENTS_REGISTER_NODE(robot_moveit::ActionClient)
