/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Basic controller for JetBot robot.
 */

#include "webots/gps.h"
#include <cstdio>
#include <webots/motor.h>
#include <webots/robot.h>
#include <motor-adapter/motor-adapter.hpp>
#include <pf-localization/pf-localization.hpp>
// #include <libraw/

#define SPEED_FACTOR 10.0

static WbDeviceTag left_motor = 0;
static WbDeviceTag right_motor = 0;

int main(int argc, char **argv) {
  wb_robot_init();

  // wb_robot_step(480);
  WbMutexRef robot_mutex = wb_robot_mutex_new();

  left_motor = wb_robot_get_device("left_wheel_hinge");
  right_motor = wb_robot_get_device("right_wheel_hinge");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, 0.2);
  wb_motor_set_velocity(right_motor, 0.2);

  int timestep = wb_robot_get_basic_time_step();

  // WbDeviceTag display = wb_robot_get_device("display");
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, 1);

  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, timestep);
  wb_receiver_set_channel(receiver, 1);

  // read integer from argv[1]
  unsigned int port = atoi(argv[1]);

  printf("Starting motor adapter on port %d\n", port-1);

  MotorAdapter motor_adapter(left_motor, right_motor, robot_mutex, port-1);
  motor_adapter.run();

  // PFLocalization loc(gps);
  PFLocalization loc(gps, receiver, emitter, port);

  printf("Starting localization on port %d\n", port);

  // create string
  std::string command = std::string("/Users/christian/Projects/vehicle-coordination/components/coordination/target/debug/picar-coordination ") + std::to_string(port);

  // set environment variable RUST_LOG to DEBUG
  // setenv("RUST_LOG", "DEBUG", 1);
  FILE *coordination = popen(command.c_str(), "r");

  printf("Starting coordination process\n");

  /* main loop */
  while (wb_robot_step(timestep) != -1) {
    loc.tick();
  };

  // kill process picar-coordination that we started before
  pclose(coordination);

  // just to be safe ...
  system("killall picar-coordination");


  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
