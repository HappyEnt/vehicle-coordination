#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <orcar_interfaces/msg/tagged_radio_packet.hpp>
#include <orcar_interfaces/Particles.pb.h>

extern "C" {
#include "particle-belief-propagation.h"
#include "util.h"
}

class BeliefPropagationNode : public rclcpp::Node
{
public:
  BeliefPropagationNode() : Node("belief_propagation")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Belief Propagation Node");

    // parameter reception_topicname
    this->declare_parameter<std::string>("receive_queue_topicname", "/ranging_radio/receive_queue");
    // declare parameter particles of type int
    this->declare_parameter<int>("particles", 2000);
    // declare paramter std_dev of type double
    this->declare_parameter<double>("receiver_deviation", 0.3);
    this->declare_parameter<std::string>("pf_method", "post_regularization");

    // register topics
    RCUTILS_LOG_INFO_NAMED("belief_propagation", "Registering topic for position estimate");
    this->position_estimate_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>
      (
       "~/robot_pose_pf/pose_estimate", 10
      );

    this->particles_publisher =
      this->create_publisher<geometry_msgs::msg::PoseArray>
      (
       "~/robot_pose_pf/particles", 10
      );

    // subscribe to receive_queue_topicname
    this->receive_queue_subscription_ =
      this->create_subscription<orcar_interfaces::msg::TaggedRadioPacket>
      (
       this->get_parameter("receive_queue_topicname").as_string(),
       10,
       std::bind(&BeliefPropagationNode::receive_queue_callback, this, std::placeholders::_1)
      );


    // initialize particle filter
    this->initialize_particle_filter();
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr position_estimate_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_publisher;
  rclcpp::Subscription<orcar_interfaces::msg::TaggedRadioPacket>::SharedPtr receive_queue_subscription_;

  enum  prediction_method {
    PREDICT_NONE,
    PREDICT_MAX_SPEED,
    PREDICT_WHEEL_SPEED, // not implemented fully yet
  } current_prediction_method;

  struct particle_filter_instance *pf_inst;

  // receive_queue_callback implementation
  void receive_queue_callback(const orcar_interfaces::msg::TaggedRadioPacket::SharedPtr msg)
  {
    // unpack payload message
    std::vector<uint8_t> payload = msg->packet.payload;
    uint32_t mac = msg->sender_mac;
    float distance = msg->range;

    // deserialize using protobuf to type Particles
    orcar::ParticleArray particles;
    particles.ParseFromArray(payload.data(), payload.size());
    // generate particles for internal c interface
    struct particle *foreign_particles =
      (struct particle *) malloc(sizeof(struct particle) * particles.particles_size());

    for (int i = 0; i < particles.particles_size(); i++) {
      foreign_particles[i].x_pos =  particles.particles(i).x();
      foreign_particles[i].y_pos =  particles.particles(i).y();
      foreign_particles[i].weight = particles.particles(i).weight();
    }

    switch (this->current_prediction_method) {
    case PREDICT_MAX_SPEED: {
      // predict_max_movement_uniform(pf_inst, 0.1, 3);
      break;
    }
    case PREDICT_WHEEL_SPEED: {
      break;
    }
    default:
      break;
    }

    struct message m = {
      .measured_distance = distance,
      .particles = foreign_particles,
      .particles_length = (size_t) particles.particles_size(),
      .type = message::DUMB_PARTICLES,
    };

    add_belief(this->pf_inst, m);

    iterate(this->pf_inst);

    // get particles from pf_inst
    struct particle *own_particles;
    int amount = get_particle_array(this->pf_inst, &own_particles);

    // create ros PoseArray from particle x,y coordinates
    geometry_msgs::msg::PoseArray particles_msg;
    particles_msg.header.frame_id = "map";
    particles_msg.header.stamp = this->now();
    particles_msg.poses.resize(amount);

    for (int i = 0; i < amount; i++) {
      particles_msg.poses[i].position.x = own_particles[i].x_pos;
      particles_msg.poses[i].position.y = own_particles[i].y_pos;
    }

    // finally publish PoseArray
    this->particles_publisher->publish(particles_msg);
  }

  void set_prediction_method(enum prediction_method method) {
    this->current_prediction_method = method;
  }

  void set_pf_method(std::string filter_type) {
    set_filter_type(this->pf_inst, POST_REGULARIZATION);
  }

  void set_particles(size_t particles) {
    set_particle_amount(this->pf_inst, particles);
  }

  void reset_filter() {
    reset_prior(this->pf_inst);
  }

  void set_receiver_deviation(double std_dev) {
    set_receiver_std_dev(this->pf_inst, std_dev);
  }

  void initialize_particle_filter() {
    create_particle_filter_instance(&this->pf_inst);

    // read from particles parameter
    set_particles(this->get_parameter("particles").as_int());

    // default to POST_REGULARIZATION
    set_pf_method(this->get_parameter("pf_method").as_string());

    pf_parallel_set_target_threads(8);

    set_receiver_deviation(this->get_parameter("receiver_deviation").as_double());
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BeliefPropagationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
