#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <custom_service/srv/fibonacci.hpp>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm"); //node, 'name of move group'
    //access and send cmd and view status of particular move_group

    std::vector<double> arm_joint_goal {1.57,-0.8,0.0,0.0,0.0,0.0};

    bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal); //returns a boolean value

    if(!arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint pos outside limits");
        return ;
    }
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

   bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS); //to reach value of joints assigned

   if(arm_plan_success)
   {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "planner succeed, moving the arm!");
        arm_move_group.move();
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
    move_robot(node);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}