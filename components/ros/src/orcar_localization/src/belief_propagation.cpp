#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <orcar_interfaces/msg/tagged_radio_packet.hpp>
#include <orcar_interfaces/Particles.pb.h>

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
    this->position_estimate_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>
      (
       "~/robot_pose_pf/pose_estimate", 10
      );

    // parameter reception_topicname
    this->declare_parameter<std::string>("receive_queue_topicname", "/ranging_radio/receive_queue");

    // subscribe to receive_queue_topicname
    this->receive_queue_subscription_ =
      this->create_subscription<orcar_interfaces::msg::TaggedRadioPacket>
      (
       this->get_parameter("receive_queue_topicname").as_string(),
       10,
       std::bind(&BeliefPropagationNode::receive_queue_callback, this, std::placeholders::_1)
      );
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr position_estimate_publisher_;
  rclcpp::Subscription<orcar_interfaces::msg::TaggedRadioPacket>::SharedPtr receive_queue_subscription_;

  // receive_queue_callback implementation
  void receive_queue_callback(const orcar_interfaces::msg::TaggedRadioPacket::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message from topic %s", msg->header.frame_id.c_str());

    // unpack payload message
    std::vector<uint8_t> payload = msg->packet.payload;
    uint32_t mac = msg->sender_mac;
    float distance = msg->range;

    // deserialize using protobuf to type Particles
    orcar::ParticleArray particles;
    particles.ParseFromArray(payload.data(), payload.size());

    // for each particle in particles print x and y coordinate
    for (int i = 0; i < particles.particles_size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Particle %d: x=%f, y=%f", i, particles.particles(i).x(), particles.particles(i).y());
    }
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BeliefPropagationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
