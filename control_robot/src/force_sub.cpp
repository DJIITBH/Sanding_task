#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

class WrenchSubscriber : public rclcpp::Node
{
public:
    WrenchSubscriber() : Node("wrench_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/jt", 10, std::bind(&WrenchSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received WrenchStamped message:");
        RCLCPP_INFO(this->get_logger(), "Force: [%.2f, %.2f, %.2f]", msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
        // RCLCPP_INFO(this->get_logger(), "Torque: [%.2f, %.2f, %.2f]", msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WrenchSubscriber>());
    rclcpp::shutdown();
    return 0;
}
