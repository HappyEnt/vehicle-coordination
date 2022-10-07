#pragma once

extern "C" {
#include "particle-belief-propagation.h"
}

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
  PFLocalization(const WbDeviceTag gps, const WbDeviceTag receiver, const WbDeviceTag transmitter, unsigned int port);
  PFLocalization(const WbDeviceTag gps, unsigned int port);
  virtual ~PFLocalization();

  enum  prediction_method {
    PREDICT_NONE,
    PREDICT_MAX_SPEED,
    PREDICT_WHEEL_SPEED, // not implemented fully yet
  };

  void tick();
  double Tick(coordination::Vec2 position_estimate, std::unordered_map<std::string, coordination::Vec2*>);
  void set_prediction_method(enum prediction_method method);
  void set_pf_method(enum filter_type method);
  void set_particles(size_t particles);
  void reset_filter();
  void set_receiver_deviation(double std_dev);

  struct particle get_last_estimate();

private:
  struct particle_filter_instance *pf_inst;
  enum prediction_method current_prediction_method;
  enum filter_type current_pf_method;

  size_t current_particles;

  WbDeviceTag transmitter;
  WbDeviceTag receiver;
  std::unique_ptr<coordination::Coordination::Stub> server_stub_;

  // grpc server stub channel
  std::shared_ptr<grpc::Channel> channel_;

  double past_time;
  struct particle most_recent_estimate;

  void tick_particle_filter();

  // for development purposes
  WbDeviceTag gps;
  // unique pointer to DataLogger
  std::unique_ptr<DataLogger> logger;
};
