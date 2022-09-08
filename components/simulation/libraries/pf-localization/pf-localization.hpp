#pragma once

#include <interface.grpc.pb.h>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/grpcpp.h>

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>

// using namespace interface;

class PFLocalization final
{
public:
  PFLocalization(const WbDeviceTag receiver, const WbDeviceTag transmitter);
  PFLocalization(const WbDeviceTag gps);
  virtual ~PFLocalization();

  void tick();
  coordination::TickResponse Tick();

private:
  WbDeviceTag transmitter;
  WbDeviceTag receiver;
  std::shared_ptr<grpc::Channel> channel;
  std::unique_ptr<coordination::Coordination::Stub> stub_;

  // for development purposes
  WbDeviceTag gps;
};

