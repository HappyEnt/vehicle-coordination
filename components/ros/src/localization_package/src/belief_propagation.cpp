#include <cstdio>
#include <rclcpp/rclcpp.hpp>
// import geometry msgs
// #include <geometry_msgs/msg/PoseWithCovarianceStamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
extern "C" {
#include "../lib/particle-belief-propagation.h"
}
// maybe useful later for gps signals
#include <sensor_msgs>


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
