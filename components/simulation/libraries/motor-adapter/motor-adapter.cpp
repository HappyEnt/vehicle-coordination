#include "motor-adapter.hpp"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/grpcpp.h>

#include <boost/log/trivial.hpp>

#define MAX_WHEEL_SPEED 20

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

  // start grpc server
  server = builder.BuildAndStart();

  // Start
  BOOST_LOG_TRIVIAL(info) << "Starting motor adapter server on " << server_address;
}

grpc::Status MotorAdapter::SetSpeed(grpc::ServerContext *context, const SetSpeedRequest *request, SetSpeedResponse *reply) {
  // set motor speed
  double left_speed = request->left();
  double right_speed = request->right();

  BOOST_LOG_TRIVIAL(info) << "Setting motor speed to " << left_speed << " and " << right_speed;

  wb_robot_mutex_lock(robot_mutex);
  wb_motor_set_velocity(left_motor, left_speed * MAX_WHEEL_SPEED);
  wb_motor_set_velocity(right_motor, right_speed * MAX_WHEEL_SPEED);
  wb_robot_mutex_unlock(robot_mutex);

  // create return message
  reply->set_success(true);
  reply->set_message("");
  return grpc::Status::OK;
}
