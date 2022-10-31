#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

extern "C" {
#include "../lib/particle-belief-propagation.h"
}

class BeliefPropagationNode : public rclcpp::Node
{
public:
  BeliefPropagationNode() : Node("belief_propagation")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Belief Propagation Node");

    // register topics
    RCUTILS_LOG_INFO_NAMED("belief_propagation", "Registering topic for position estimate");
    this->position_estimate_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/robot_pose_pf/pose_estimate", 10);
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr position_estimate_publisher_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BeliefPropagationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
