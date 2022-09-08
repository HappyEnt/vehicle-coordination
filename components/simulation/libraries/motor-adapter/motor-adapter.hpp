#pragma once

#include <picar.grpc.pb.h>

#include <webots/motor.h>
#include <webots/robot.h>

using namespace picar;

class MotorAdapter final : public Picar::Service
{
public:
  MotorAdapter(const WbDeviceTag left_motor, WbDeviceTag right_motor);
  virtual ~MotorAdapter();

  grpc::Status SetSpeed(grpc::ServerContext* context, const SetSpeedRequest* request,
                        SetSpeedResponse* reply) override;

  void run();

private:
  WbDeviceTag left_motor;
  WbDeviceTag right_motor;
  std::string server_address;
  // picar service
  // picar::PicarService picar_service;
  void start_grpc_service();
};

