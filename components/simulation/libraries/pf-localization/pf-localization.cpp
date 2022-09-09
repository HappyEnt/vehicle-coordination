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

#define PARTICLES 2000

PFLocalization::PFLocalization(const WbDeviceTag gps) : gps(gps) {
  past_time = wb_robot_get_time();

  std::string target_str = "0.0.0.0:50052";

  // try to create grpc channel
  grpc::ChannelArguments channel_args;
  channel_args.SetMaxReceiveMessageSize(-1);

  // try to create grpc channel and catch errors
  channel_ = grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials());
  server_stub_ = coordination::Coordination::NewStub(channel_);
}

PFLocalization::PFLocalization(const WbDeviceTag receiver, const WbDeviceTag transmitter) : receiver(receiver), transmitter(transmitter) {
  past_time = wb_robot_get_time();

  std::string target_str = "0.0.0.0:50052";

  // try to create grpc channel
  grpc::ChannelArguments channel_args;
  channel_args.SetMaxReceiveMessageSize(-1);

  // try to create grpc channel and catch errors
  channel_ = grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials());
  server_stub_ = coordination::Coordination::NewStub(channel_);

  // initialize particle filter
  create_particle_filter_instance(&pf_inst);

  /* set_particle_array(*pf_inst, own_particles, PARTICLES); */
  set_particle_amount(pf_inst, PARTICLES);

  set_filter_type(pf_inst, POST_REGULARIZATION);

  pf_parallel_set_target_threads(8);

  set_receiver_std_dev(pf_inst, 0.4);

  logger = std::make_unique<DataLogger>(DataLogger(wb_robot_get_name(), PARTICLES));
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

  while(queue_length > 0) {
    const char *data = (char *) wb_receiver_get_data(receiver);
    int data_len = wb_receiver_get_data_size(receiver);
    size_t particles = data_len / sizeof(struct particle);
    double rssi = wb_receiver_get_signal_strength(receiver);

    struct particle *foreign_particles = (struct particle *) malloc(sizeof(struct particle) * particles);
    double measurement = sqrt((double)1.0 / rssi);

    memcpy(foreign_particles, data, sizeof(struct particle) * particles);

    struct message m = {
      .measured_distance = measurement,
      .particles = foreign_particles,
      .particles_length = particles,
      .type = message::DUMB_PARTICLES,
    };

    add_belief(pf_inst, m);

    free(foreign_particles);
    wb_receiver_next_packet(receiver);
    queue_length--;

    if(queue_length <= 0) {

      predict_max_movement_uniform(pf_inst, 0.1, 3);

      iterate(pf_inst);

      struct particle mean = estimate_position(pf_inst);

      if(channel_->GetState(true) == GRPC_CHANNEL_READY) {
        coordination::Vec2 position_estimate;
        position_estimate.set_x(mean.x_pos);
        position_estimate.set_y(mean.y_pos);
        Tick(position_estimate);
      }

      struct particle *particles;
      int amount = get_particle_array(pf_inst, &particles);
      logger->write_particles(particles, amount);
    }
  }
}

void PFLocalization::tick() {
  const double dt = wb_robot_get_time() - past_time;
  if (dt > 0.1) {
    tick_particle_filter();
    past_time = wb_robot_get_time();
  }
}

double PFLocalization::Tick(coordination::Vec2 position_estimate) {
  // coordination::Vec2 position;

  // position.set_x(wb_gps_get_values(gps)[0]);
  // position.set_y(wb_gps_get_values(gps)[1]);

  BOOST_LOG_TRIVIAL(info) << "Sending position_estimate: " << position_estimate.x() << ", " << position_estimate.y();

  // See https://github.com/protocolbuffers/protobuf/issues/435 ????
  coordination::TickRequest *request = new(coordination::TickRequest);
  request->set_id(6);
  request->set_allocated_position(&position_estimate);
  request->set_radius(0.2);
  request->set_confidence(0.0);

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
