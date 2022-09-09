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

PFLocalization::~PFLocalization() {}


void PFLocalization::tick() {
  const double dt = wb_robot_get_time() - past_time;

  if (dt > 0.1) {
    if(channel_->GetState(true) == GRPC_CHANNEL_READY) {
      Tick();
    }
    past_time = wb_robot_get_time();
  }
}

double PFLocalization::Tick() {
  coordination::Vec2 position;

  position.set_x(wb_gps_get_values(gps)[0]);
  position.set_y(wb_gps_get_values(gps)[1]);

  BOOST_LOG_TRIVIAL(info) << "Sending position: " << position.x() << ", " << position.y();

  // See https://github.com/protocolbuffers/protobuf/issues/435 ????
  coordination::TickRequest *request = new(coordination::TickRequest);
  request->set_id(6);
  request->set_allocated_position(&position);
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
