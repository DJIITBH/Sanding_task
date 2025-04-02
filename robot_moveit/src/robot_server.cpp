#include<rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <custom_service/action/plan_robot.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include<thread>

using namespace std::placeholders;

namespace robot_moveit
{
class RobotServer : public rclcpp::Node
{
public:
    explicit RobotServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()): Node("robot_server_node", options)
    {
        action_server_ = rclcpp_action::create_server<custom_service::action::PlanRobot>(
            this, "robot_server", std::bind(&RobotServer::goal_callback, this, _1, _2), 
            std::bind(&RobotServer::cancel_callback, this, _1), 
            std::bind(&RobotServer::accept_callback, this, _1)
        );
        subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/jt", 10, std::bind(&RobotServer::topic_callback, this, std::placeholders::_1));        //goal call back is called when action server receives a new goal 
        // then it decides whether to accept it or reject it then accept call back is executed!
        // if it receives a cancellation request then goal is cancelled using cancel callback function


        RCLCPP_INFO(this->get_logger(),"starting the action server");

    }
private:
    rclcpp_action::Server<custom_service::action::PlanRobot>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
    double force = 0;

    void topic_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received WrenchStamped message:");
        // RCLCPP_INFO(this->get_logger(), "Force: [%.2f, %.2f, %.2f]", msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
        // RCLCPP_INFO(this->get_logger(), "Torque: [%.2f, %.2f, %.2f]", msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
        force = msg->wrench.force.x;
    }

    // member functions..

    void marker_pub(const std::shared_ptr<rclcpp::Node>& node,
        const std::vector<std::array<double, 3>>& pts,
        const std::string& frame_id )
        {
            auto publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
                "visualization_marker_array", 10);
        
            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker marker;
        
            for (size_t i = 0; i < pts.size(); i++) {
                marker.header.frame_id = frame_id;
                marker.header.stamp = node->now();
                marker.ns = "marker_points";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
        
                marker.pose.position.x = pts[i][0];  // Note: You may want to remove these sign flips
                marker.pose.position.y = pts[i][1];
                marker.pose.position.z = pts[i][2];
        
                marker.scale.x = 0.08;
                marker.scale.y = 0.08;
                marker.scale.z = 0.08;
        
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
        
                marker.lifetime = rclcpp::Duration::from_seconds(0);
                marker_array.markers.push_back(marker);
            }
        
            publisher->publish(marker_array);
        }
    
    
    bool move_robot(const std::shared_ptr<rclcpp::Node> node)
    {
        auto move_group_arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); //node, 'name of move group'
        //access and send cmd and view status of particular move_group
    
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
        geometry_msgs::msg::Pose target_pose1;
        std::vector<std::array<double, 3>> pts;
    
        target_pose1.orientation.x = -1.0;
        target_pose1.orientation.y = 0.00;
        target_pose1.orientation.z = 0.00;
        target_pose1.orientation.w = 0.00;
        target_pose1.position.x = 0.35;
        target_pose1.position.y = 0.3;
        target_pose1.position.z = 0.0;
        move_group_arm.setPoseTarget(target_pose1);
    
        bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::core::MoveItErrorCode::SUCCESS);
    
        move_group_arm.execute(my_plan_arm);
    
        // Approach
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approach to object!");
    
        std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    
        for (int i = 0;i<6;i++){
            target_pose1.position.y = -0.43;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
            target_pose1.position.x = target_pose1.position.x + 0.05;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
            target_pose1.position.y = 0.3;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
            target_pose1.position.x = target_pose1.position.x + 0.05;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
        }
        marker_pub(node,pts, "world");
        
    
        moveit_msgs::msg::RobotTrajectory trajectory_approach;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
    
        double fraction = move_group_arm.computeCartesianPath(
            approach_waypoints, eef_step, jump_threshold, trajectory_approach);
    
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "yaha hu!");
        bool arm_plan_success = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

        if(arm_plan_success)
        {
             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, Executing Sanding!");
             bool flag = true;
             move_group_arm.execute(trajectory_approach);
             return flag;
        }
        else
        {
             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed!");
             bool flag = false;
             return flag;
        }


        // std::thread execution_thread([this]() {
        //     move_group_arm.asyncExecute(trajectory_approach);
        // });
    }
    
    bool quality_check(const std::shared_ptr<rclcpp::Node> node)
    {
        auto move_group_arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); //node, 'name of move group'
        //access and send cmd and view status of particular move_group
    
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    
        geometry_msgs::msg::Pose target_pose1;
        std::vector<std::array<double, 3>> pts;
    
        target_pose1.orientation.x = -1.0;
        target_pose1.orientation.y = 0.00;
        target_pose1.orientation.z = 0.00;
        target_pose1.orientation.w = 0.00;
        target_pose1.position.x = 0.35;
        target_pose1.position.y = 0.3;
        target_pose1.position.z = 0.0;
        move_group_arm.setPoseTarget(target_pose1);
    
        bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::core::MoveItErrorCode::SUCCESS);
    
        move_group_arm.execute(my_plan_arm);
    
        // Approach
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approach to object!");
    
        std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    
            target_pose1.position.y = -0.43;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
            target_pose1.position.x = 0.95;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
            target_pose1.position.y = 0.3;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
            target_pose1.position.x = 0.35;
            pts.push_back({target_pose1.position.x, target_pose1.position.y, target_pose1.position.z});
            approach_waypoints.push_back(target_pose1);
        
        marker_pub(node,pts, "world");
        
    
        moveit_msgs::msg::RobotTrajectory trajectory_approach;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
    
        double fraction = move_group_arm.computeCartesianPath(
            approach_waypoints, eef_step, jump_threshold, trajectory_approach);
    

        bool arm_plan_success = (move_group_arm.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

        if(arm_plan_success)
        {
             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, Doing Quality Check!");
             bool flag = true;
             move_group_arm.execute(trajectory_approach);
             return flag;
        }
        else
        {
             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed!");
             bool flag = false;
             return flag;
        }
    
    }
    bool home_pos(const std::shared_ptr<rclcpp::Node> node)
    {
    auto move_group_arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); //node, 'name of move group'
    //access and send cmd and view status of particular move_group

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    std::vector<double> arm_joint_goal {0.0,-1.57,0.0,-1.57,0.0,0.0};

    bool arm_within_bounds = move_group_arm.setJointValueTarget(arm_joint_goal); //returns a boolean value

    if(!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint pos outside limits");
        return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

    bool arm_plan_success = (move_group_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

   if(arm_plan_success)
   {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, Going to safe position!");
        bool flag = true;
        move_group_arm.move();
        return flag;
   }
   else
   {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed!");
        bool flag = false;
        return flag;
   }

  }
    // fucntions end ...

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const custom_service::action::PlanRobot::Goal>goal)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"), 
            "Received goal request with goal id %d", 
            goal->task_number
        );
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        //we are accepting the goal everytime it comes...
    }
    void accept_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::PlanRobot>> goal_handle)
    {
        //in order to avoid blocking of current thread we will run the logic in a different thread
        //not to keep client blocked for too long while waiting for server 
        // to execute and finsh the accepted callback function
        std::thread{std::bind(&RobotServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::PlanRobot>> goal_handle)
    {
        bool flag = false;
        rclcpp::Rate loop_rate(1);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "executing goal..");
        // TO MOVE THE ROBOT WE NEED MOVEIT2 API
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        std::vector<double> arm_joint_goal;

        auto feedback = std::make_shared<custom_service::action::PlanRobot::Feedback>();
        auto& force_val = feedback->force;

        force_val = force;

        if(goal_handle->get_goal()->task_number == 0)
        {
            flag = home_pos(shared_from_this());
            //home location..
        }
        else if(goal_handle->get_goal()->task_number == 1)
        {
            flag = quality_check(shared_from_this()); 
            //random location
        }
        else if(goal_handle->get_goal()->task_number == 2)
        {
            flag = move_robot(shared_from_this()); 
            //random location
        }
        else{
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "invalid Robot number..");

        }

        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "force feedback!");
        // loop_rate.sleep();

        auto result = std::make_shared<custom_service::action::PlanRobot::Result>();
        result->success = flag;
        if (flag)
        {
            goal_handle->succeed(result);  // Client sees SUCCESS
        }
        else
        {
            goal_handle->abort(result);  // Explicit failure

        }
        
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal succeeded");

    }
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_service::action::PlanRobot>> goal_handle)
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

RCLCPP_COMPONENTS_REGISTER_NODE(robot_moveit::RobotServer)
