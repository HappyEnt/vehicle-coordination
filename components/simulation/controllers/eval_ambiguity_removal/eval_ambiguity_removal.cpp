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

#include "particle-belief-propagation.h"
#include "webots/gps.h"
#include <cstdio>
#include <webots/motor.h>
#include <webots/robot.h>
#include <motor-adapter/motor-adapter.hpp>
#include <pf-localization/pf-localization.hpp>

#include <visualization/data_logger.hpp>

#define SPEED_FACTOR 10.0

static WbDeviceTag left_motor = 0;
static WbDeviceTag right_motor = 0;

int main(int argc, char **argv) {
  wb_robot_init();


  std::string NodeName(wb_robot_get_name());

  WbMutexRef robot_mutex = wb_robot_mutex_new();

  left_motor = wb_robot_get_device("left_wheel_hinge");
  right_motor = wb_robot_get_device("right_wheel_hinge");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, atof(argv[1]));
  wb_motor_set_velocity(right_motor, atof(argv[2]));

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
  unsigned int port = 1200;

  // PFLocalization loc(gps);
  PFLocalization loc(gps, receiver, emitter, port);
  loc.set_prediction_method(PFLocalization::PREDICT_MAX_SPEED);
  loc.set_pf_method(POST_REGULARIZATION);
  loc.set_receiver_deviation(0.6);

  printf("Starting localization on port %d\n", port);

  while (wb_robot_step(timestep) != -1) {
    loc.tick();
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
