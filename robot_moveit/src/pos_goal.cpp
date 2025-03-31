#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/twist.hpp"

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


void move_robot(const std::shared_ptr<rclcpp::Node> node)
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

    for (int i = 0;i<7;i++){
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

    move_group_arm.execute(trajectory_approach);

    std::vector<double> arm_joint_goal {0.0,-1.57,0.0,-1.57,0.0,0.0};

    bool arm_within_bounds = move_group_arm.setJointValueTarget(arm_joint_goal); //returns a boolean value

    if(!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint pos outside limits");
        return ;
    }
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

    bool arm_plan_success = (move_group_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

   if(arm_plan_success)
   {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, moving the arm!");
        move_group_arm.move();
   }
   else
   {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed!");
        return;
   }

}

void quality_check(const std::shared_ptr<rclcpp::Node> node)
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

    move_group_arm.execute(trajectory_approach);

    std::vector<double> arm_joint_goal {0.0,-1.57,0.0,-1.57,0.0,0.0};

    bool arm_within_bounds = move_group_arm.setJointValueTarget(arm_joint_goal); //returns a boolean value

    if(!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint pos outside limits");
        return ;
    }
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

    bool arm_plan_success = (move_group_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

   if(arm_plan_success)
   {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, moving the arm!");
        move_group_arm.move();
   }
   else
   {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed!");
        return;
   }

}

void home_pos(const std::shared_ptr<rclcpp::Node> node)
{
    auto move_group_arm = moveit::planning_interface::MoveGroupInterface(node, "arm"); //node, 'name of move group'
    //access and send cmd and view status of particular move_group

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    std::vector<double> arm_joint_goal {0.0,-1.57,0.0,-1.57,0.0,0.0};

    bool arm_within_bounds = move_group_arm.setJointValueTarget(arm_joint_goal); //returns a boolean value

    if(!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint pos outside limits");
        return ;
    }
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

    bool arm_plan_success = (move_group_arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

   if(arm_plan_success)
   {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, moving the arm!");
        move_group_arm.move();
   }
   else
   {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "planner failed!");
        return;
   }

}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
    quality_check(node);
    move_robot(node);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}