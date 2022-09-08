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

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  int timestep = wb_robot_get_basic_time_step();

  // WbDeviceTag display = wb_robot_get_device("display");
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  MotorAdapter motor_adapter(left_motor, right_motor, robot_mutex);
  motor_adapter.run();

  PFLocalization loc(gps);

  /* main loop */
  while (wb_robot_step(timestep) != -1) {
    // step_forward();
    // wb_robot_step(960);
    // printf("stepping\n");
    loc.tick();
    // motor_adapter.tick();
    // step_backward();
    // wb_robot_step(960);
    // step_left();
    // wb_robot_step(960);
    // step_right();
    // wb_robot_step(960);
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
