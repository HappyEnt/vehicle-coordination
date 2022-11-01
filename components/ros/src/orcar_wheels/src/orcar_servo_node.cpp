#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ServoWheelNode : public rclcpp::Node
{
public:
  ServoWheelNode() : Node("servo_wheel")
  {
    RCLCPP_INFO(this->get_logger(), "Starting ServoWheel Node");

    // read wheel radius from robot urdf?? TODO

    // register topics
    this->cmd_vel_ =
      this->create_subscription<geometry_msgs::msg::Twist>
      (
       "~/cmd_vel",
       10,
       std::bind(&ServoWheelNode::cmd_vel_callback, this, std::placeholders::_1)
      );
  }
private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoWheelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
