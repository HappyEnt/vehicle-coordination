#pragma once

#include <picar.grpc.pb.h>

#include <webots/motor.h>
#include <webots/robot.h>

using namespace picar;

class MotorAdapter final : public Picar::Service
{
public:
  // MotorAdapter(const WbDeviceTag left_motor, WbDeviceTag right_motor);

  MotorAdapter(const WbDeviceTag left_motor, WbDeviceTag right_motor, WbMutexRef robot_mutex) : left_motor(left_motor), right_motor(right_motor), robot_mutex(robot_mutex) {
    server_address = std::string("0.0.0.0:50051");
  }
  virtual ~MotorAdapter();

  grpc::Status SetSpeed(grpc::ServerContext* context, const SetSpeedRequest* request,
                        SetSpeedResponse* reply) override;

  void run();

private:
  WbDeviceTag left_motor;
  WbDeviceTag right_motor;
  WbMutexRef robot_mutex;

  std::string server_address;

  std::unique_ptr<grpc::Server> server;

  // picar service
  // picar::PicarService picar_service;
  void start_grpc_service();
};

