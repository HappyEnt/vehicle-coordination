#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class BeliefPropagation : public rclcpp::Node
{
public:
  BeliefPropagation() : Node("belief_propagation")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Belief Propagation Node");
  }
private:
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BeliefPropagation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
