#include "pf-localization.hpp"
#include "interface.grpc.pb.h"
#include "interface.pb.h"

#include <grpc/grpc.h>
#include <iomanip>
#include <memory>
#include <webots/gps.h>

#include <boost/log/trivial.hpp>

extern "C" {
#include <particle-belief-propagation.h>
#include <util.h>
}

PFLocalization::PFLocalization(const WbDeviceTag gps, unsigned int port) : gps(gps) {
  past_time = wb_robot_get_time();

  std::string target_str = std::string("0.0.0.0:") + std::to_string(port);

  // try to create grpc channel
  grpc::ChannelArguments channel_args;
  channel_args.SetMaxReceiveMessageSize(-1);

  // try to create grpc channel and catch errors
  channel_ = grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials());
  server_stub_ = coordination::Coordination::NewStub(channel_);

  // default prediction method is max speed
  this->current_prediction_method = PREDICT_MAX_SPEED;
}

void PFLocalization::set_prediction_method(enum prediction_method method) {
  this->current_prediction_method = method;
}

void PFLocalization::set_pf_method(enum filter_type method) {
  this->current_pf_method = method;
  set_filter_type(pf_inst, method);
}

void PFLocalization::set_particles(size_t particles) {
  this->current_particles = particles;
  set_particle_amount(pf_inst, particles);
}

void PFLocalization::reset_filter() {
  reset_prior(this->pf_inst);
}

void PFLocalization::set_receiver_deviation(double std_dev) {
  set_receiver_std_dev(pf_inst, std_dev);
}

PFLocalization::PFLocalization(const WbDeviceTag gps, const WbDeviceTag receiver, const WbDeviceTag transmitter, unsigned int port) : gps(gps), receiver(receiver), transmitter(transmitter) {
  this->transmitter = transmitter;
  this->receiver = receiver;

  past_time = wb_robot_get_time();

  std::string target_str = std::string("0.0.0.0:") + std::to_string(port);

  // try to create grpc channel
  grpc::ChannelArguments channel_args;
  channel_args.SetMaxReceiveMessageSize(-1);

  // try to create grpc channel and catch errors
  channel_ = grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials());
  server_stub_ = coordination::Coordination::NewStub(channel_);

  // initialize particle filter
  create_particle_filter_instance(&pf_inst);

  // default to 500 particles
  set_particles(2000);

  // default to POST_REGULARIZATION
  set_pf_method(POST_REGULARIZATION);

  pf_parallel_set_target_threads(8);

  set_receiver_deviation(0.3);

  logger = std::unique_ptr<DataLogger>(new DataLogger(wb_robot_get_name(), current_particles));
  logger->register_node_with_vis();
}

PFLocalization::~PFLocalization() {}

void deinit_filter(struct particle_filter_instance *pf_inst) {
  if(pf_inst != NULL) {
    destroy_particle_filter_instance(pf_inst);
    pf_inst = NULL;
  }
}

void PFLocalization::tick_particle_filter() {
  int queue_length = wb_receiver_get_queue_length(receiver);

  // create hashmap with a string as key and coordination::Vec2* as value
  std::unordered_map<std::string, coordination::Vec2*> foreign_estimate;

  while(queue_length > 0) {
    const char *data = (char *) wb_receiver_get_data(receiver);
    // create struct com_message from received data
    int data_len = wb_receiver_get_data_size(receiver);

    char node_name[200];
    strcpy(node_name, data);

    size_t particles = (data_len - 200) / sizeof(struct particle);
    double rssi = wb_receiver_get_signal_strength(receiver);

    struct particle *foreign_particles = (struct particle *) malloc(sizeof(struct particle) * particles);
    double measurement = sqrt((double)1.0 / rssi);

    memcpy(foreign_particles, data+200, sizeof(struct particle) * particles);

    struct message m = {
      .measured_distance = measurement,
      .particles = foreign_particles,
      .particles_length = particles,
      .type = message::DUMB_PARTICLES,
    };

    struct particle other_mean = empirical_mean(foreign_particles, particles);

    // create coordination::Vec2 from other_mean
    coordination::Vec2 *other_mean_vec = new coordination::Vec2();
    other_mean_vec->set_x(other_mean.x_pos);
    other_mean_vec->set_y(other_mean.y_pos);

    // add other_mean_vec at key node_name to foreign_estimate
    foreign_estimate.insert(std::make_pair(std::string(node_name), other_mean_vec));

    add_belief(pf_inst, m);

    free(foreign_particles);
    wb_receiver_next_packet(receiver);
    queue_length--;

    if(queue_length <= 0) {
      switch (current_prediction_method) {
      case PREDICT_MAX_SPEED: {
        predict_max_movement_uniform(pf_inst, 0.1, 3);
        break;
      }
      case PREDICT_WHEEL_SPEED: {
        BOOST_LOG_TRIVIAL(info) << "PREDICT_WHEEL_SPEED not implemented yet";
        break;
      }
      default:
        break;
      }

      iterate(pf_inst);

      struct particle mean = estimate_position(pf_inst);

      most_recent_estimate = mean;

      coordination::Vec2 position_estimate;
      position_estimate.set_x(mean.x_pos);
      position_estimate.set_y(mean.y_pos);

      if(channel_->GetState(true) == GRPC_CHANNEL_READY) {
        Tick(position_estimate, foreign_estimate);
      }

#ifdef USE_GROUND_TRUTH
      struct particle p;
      p.x_pos = wb_gps_get_values(gps)[0];
      p.y_pos = wb_gps_get_values(gps)[1];
      p.weight = 1.0;
      char data[sizeof(struct particle)+200];
      strcpy(data, wb_robot_get_name());
      memcpy(data+200, &p, sizeof(struct particle));
      wb_emitter_send(transmitter, (char*) data, sizeof(data));
#else
      struct particle *particles;
      int amount = get_particle_array(pf_inst, &particles);
      char data[sizeof(particle)*amount+200];
      strcpy(data, wb_robot_get_name());
      memcpy(data+200, particles, sizeof(particle)*amount);
      wb_emitter_send(transmitter, (char*) data, sizeof(data));
#endif

      logger->write_particles(particles, amount);
    }
  }
}

struct particle PFLocalization::get_last_estimate() {
  return most_recent_estimate;
}

void PFLocalization::tick() {
  const double dt = wb_robot_get_time() - past_time;
  if (dt > 0.25) {
    tick_particle_filter();
    past_time = wb_robot_get_time();
  }
}

double PFLocalization::Tick(coordination::Vec2 position_estimate, std::unordered_map<std::string, coordination::Vec2*> other_positions) {
  // See https://github.com/protocolbuffers/protobuf/issues/435 ????
  coordination::TickRequest *request = new(coordination::TickRequest);
  request->set_id(6);

#ifdef USE_GROUND_TRUTH
  coordination::Vec2 ground_truth_position;

  ground_truth_position.set_x(wb_gps_get_values(gps)[0]);
  ground_truth_position.set_y(wb_gps_get_values(gps)[1]);

  request->set_allocated_position(&ground_truth_position);
#else
  request->set_allocated_position(&position_estimate);
#endif

  request->set_radius(0.25);
  request->set_confidence(0.0);

  // for every key,value pair in forein_positions do
  unsigned int id_counter = 7;
  for (auto const& [key, val] : other_positions) {
    // add val to request->add_other_positions()

    // derive from string key a unique integer
    unsigned int id = std::hash<std::string>{}(key);

    // create std::unique_ptr from other_position
    coordination::TickRequest::Participant *participant = request->add_others();
    participant->set_id(id_counter++);
    participant->set_radius(0.25);
    participant->set_confidence(0.0);
    participant->set_allocated_position(val);
  }

  // Container for the data we expect from the server.
  coordination::TickResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = server_stub_->Tick(&context, *request, &reply);

  // Act upon its status.
  if (status.ok()) {
    free(request);
    return 1.0;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
}
