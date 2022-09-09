#pragma once

#include <interface.grpc.pb.h>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/grpcpp.h>

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>


#include <visualization/data_logger.hpp>

// using namespace interface;

class PFLocalization final
{
public:
  PFLocalization(const WbDeviceTag receiver, const WbDeviceTag transmitter);
  PFLocalization(const WbDeviceTag gps);
  virtual ~PFLocalization();

  void tick();
  double Tick(coordination::Vec2 position_estimate);

private:
  struct particle_filter_instance *pf_inst;

  WbDeviceTag transmitter;
  WbDeviceTag receiver;
  std::unique_ptr<coordination::Coordination::Stub> server_stub_;

  // grpc server stub channel
  std::shared_ptr<grpc::Channel> channel_;

  double past_time;

  void tick_particle_filter();

  // for development purposes
  WbDeviceTag gps;
  // unique pointer to DataLogger
  std::unique_ptr<DataLogger> logger;
};
