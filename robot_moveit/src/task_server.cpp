#include<rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <custom_service/action/robot_task.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include<thread>

using namespace std::placeholders;

namespace robot_moveit
{
class TaskServer : public rclcpp::Node
{
public:
    explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("task_server_node", options)
    {
        action_server_ = rclcpp_action::create_server<custom_service::action::RobotTask>(
            this, "task_server", std::bind(&TaskServer::goal_callback, this, _1, _2), 
            std::bind(&TaskServer::cancel_callback, this, _1), 
            std::bind(&TaskServer::accept_callback, this, _1)
        );
        //goal call back is called when action server receives a new goal 
        // then it decides whether to accept it or reject it then accept call back is executed!
        // if it receives a cancellation request then goal is cancelled using cancel callback function
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"starting the action server");

    }
private:
    rclcpp_action::Server<custom_service::action::RobotTask>::SharedPtr action_server_;

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const custom_service::action::RobotTask::Goal>goal)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), 
            "Received goal request with goal id %d", 
            goal->task_number
        );
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        //we are accepting the goal everytime it comes...
    }
    void accept_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::RobotTask>> goal_handle)
    {
        //in order to avoid blocking of current thread we will run the logic in a different thread
        //not to keep client blocked for too long while waiting for server 
        // to execute and finsh the accepted callback function
        std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::RobotTask>> goal_handle)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executing goal..");
        // TO MOVE THE ROBOT WE NEED MOVEIT2 API
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        std::vector<double> arm_joint_goal;

        if(goal_handle->get_goal()->task_number == 0)
        {
            arm_joint_goal = {0.0,-1.57,0.0,-1.57,0.0,0.0}; 
            //home location..
        }
        else if(goal_handle->get_goal()->task_number == 1)
        {
            arm_joint_goal = {0.0,-0.5,0.5,-1.57,0.0,0.0}; 
            //random location
        }
        else if(goal_handle->get_goal()->task_number == 2)
        {
            arm_joint_goal = {0.0,-1.57,0.0,0.0,0.0,0.0}; 
            //random location
        }
        else{
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "invalid task number..");

        }

        bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
        //check if both values are actually reachable or not

        if (!arm_within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "target joint position was outside limits..");
            return;

        }
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(arm_plan_success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, moving arm..");
            arm_move_group.move();
            //execute the trajectory.
        }
        else{
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed");
            return ;

        }
        auto result = std::make_shared<custom_service::action::RobotTask::Result>();
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal succeeded");



    }
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::RobotTask>> goal_handle)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "received request to cancel the goal!");
        //action server accepted goal cancellation.
        //robot should stop immediately
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");

        arm_move_group.stop();
        //stop the robot through moveit.

        return rclcpp_action::CancelResponse::ACCEPT;


    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_moveit::TaskServer)
