#include "motor-adapter.hpp"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/grpcpp.h>

#include <boost/log/trivial.hpp>

MotorAdapter::MotorAdapter(const WbDeviceTag left_motor, WbDeviceTag right_motor) : left_motor(left_motor), right_motor(right_motor) {
  server_address = std::string("0.0.0.0:50051");
}

MotorAdapter::~MotorAdapter() {
}

void MotorAdapter::run() {
  grpc::EnableDefaultHealthCheckService(true);

  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  grpc::ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(this);

  // Start
  BOOST_LOG_TRIVIAL(info) << "Starting motor adapter server on " << server_address;

  // start grpc server
  builder.BuildAndStart();
}

grpc::Status MotorAdapter::SetSpeed(grpc::ServerContext *context, const SetSpeedRequest *request, SetSpeedResponse *reply) {
  // set motor speed
  double left_speed = request->left();
  double right_speed = request->right();

  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);

  // create return message
  reply->set_success(true);
  reply->set_message("");
  return grpc::Status::OK;
}
