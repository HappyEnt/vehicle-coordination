#include "pf-localization.hpp"
#include "interface.pb.h"

#include <webots/gps.h>

#include <boost/log/trivial.hpp>

extern "C" {
#include <particle-belief-propagation.h>
#include <util.h>
}

PFLocalization::PFLocalization(const WbDeviceTag gps) : gps(gps) {}

void PFLocalization::tick() {

}

coordination::TickResponse PFLocalization::Tick() {
  coordination::Vec2 position;

  position.set_x(wb_gps_get_values(gps)[0]);
  position.set_y(wb_gps_get_values(gps)[0]);

  // Data we are sending to the server.
  coordination::TickRequest request;
  request.set_id(0);
  request.set_allocated_position(&position);

  // Container for the data we expect from the server.
  coordination::TickResponse reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = stub_->Tick(&context, request, &reply);

  // Act upon its status.
  if (status.ok()) {
    return reply;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
  }
}
