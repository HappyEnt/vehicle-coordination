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

struct setup {
  double duration;
  size_t repetitions;
  enum filter_type filter;
  size_t particles;
} sim_configuration[6] = {
  {
    .duration = 5,
    .repetitions = 100,
    .filter = POST_REGULARIZATION,
    .particles = 250,
  },
  {
    .duration = 5,
    .repetitions = 100,
    .filter = POST_REGULARIZATION,
    .particles = 500,
  },
  {
    .duration = 5,
    .repetitions = 100,
    .filter = POST_REGULARIZATION,
    .particles = 1000,
  },
  {
    .duration = 5,
    .repetitions = 100,
    .filter = SIR_ROUGHENING,
    .particles = 250,
  },
  {
    .duration = 5,
    .repetitions = 100,
    .filter = SIR_ROUGHENING,
    .particles = 500,
  },
  {
    .duration = 5,
    .repetitions = 100,
    .filter = SIR_ROUGHENING,
    .particles = 1000,
  },
};


// struct setup {
//   double duration;
//   size_t repetitions;
//   enum filter_type filter;
//   size_t particles;
// } sim_configuration[6] = {
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 20,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 500,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PRE_REGULARIZATION,
//     .particles = 250,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PRE_REGULARIZATION,
//     .particles = 500,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 125,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 250,
//   },
// };

// struct setup {
//   double duration;
//   size_t repetitions;
//   enum filter_type filter;
//   size_t particles;
// } sim_configuration[6] = {
//   // {
//   //   .duration = 4,
//   //   .repetitions = 20,
//   //   .filter = PROGRESSIVE_PRE_REGULARIZATION,
//   //   .particles = 20,
//   // },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 40,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 80,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 160,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 320,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 640,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PROGRESSIVE_PRE_REGULARIZATION,
//     .particles = 1280,
//   },
// };

// struct setup {
//   double duration;
//   size_t repetitions;
//   enum filter_type filter;
//   size_t particles;
// } sim_configuration[4] = {
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PRE_REGULARIZATION,
//     .particles = 160,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PRE_REGULARIZATION,
//     .particles = 320,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PRE_REGULARIZATION,
//     .particles = 640,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = PRE_REGULARIZATION,
//     .particles = 1280,
//   },
// };


// struct setup {
//   double duration;
//   size_t repetitions;
//   enum filter_type filter;
//   size_t particles;
// } sim_configuration[7] = {
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 20,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 40,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 80,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 160,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 320,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 640,
//   },
//   {
//     .duration = 4,
//     .repetitions = 20,
//     .filter = POST_REGULARIZATION,
//     .particles = 1280,
//   },
// }

#define SPEED_FACTOR 10.0

static WbDeviceTag left_motor = 0;
static WbDeviceTag right_motor = 0;


void set_next_setup(struct setup conf,   PFLocalization &loc) {
  loc.set_particles(conf.particles);
  loc.set_pf_method(conf.filter);
  loc.reset_filter();
};

void write_stats(std::shared_ptr<std::ofstream> output_file, struct setup conf, struct particle estimated_location, struct particle ground_truth) {
  double error = sqrt(pow(estimated_location.x_pos - ground_truth.x_pos, 2) + pow(estimated_location.y_pos - ground_truth.y_pos, 2));
  *output_file << conf.particles << "," << conf.filter << "," << error << std::endl;
}


int main(int argc, char **argv) {
  wb_robot_init();


  std::string NodeName( wb_robot_get_name() );
  // create ofstream for error_stats_file
  std::shared_ptr<std::ofstream> error_stats_file;
  // create basic_ofstream for error_stats_file
  error_stats_file = std::shared_ptr<std::ofstream>(new std::ofstream());
  error_stats_file->open(std::string("error-").append(NodeName).append(".csv"), std::ios::out);

  // wb_robot_step(480);
  WbMutexRef robot_mutex = wb_robot_mutex_new();

  left_motor = wb_robot_get_device("left_wheel_hinge");
  right_motor = wb_robot_get_device("right_wheel_hinge");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

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

  // PFLocalization loc(gps);
  PFLocalization loc(gps, receiver, emitter, port);
  loc.set_prediction_method(PFLocalization::PREDICT_NONE);

  printf("Starting localization on port %d\n", port);

  size_t current_setup = 0;

  size_t current_setup_repetitions = sim_configuration[current_setup].repetitions;
  size_t repetition_counter = 0;
  double current_setup_duration = sim_configuration[current_setup].duration;
  double time_acc = 0;
  double dt = wb_robot_get_basic_time_step()/1000;

  std::unique_ptr<DataLogger> logger;
  logger = std::unique_ptr<DataLogger>(new DataLogger(NodeName, sim_configuration[current_setup].particles));
  logger->register_node_with_vis();

  /* main loop */
  while (wb_robot_step(timestep) != -1) {
    time_acc += dt;

    if(current_setup < sizeof(sim_configuration)/sizeof(struct setup) && time_acc > current_setup_duration) {
      printf("Setting up next configuration\n");

      if(time_acc > sim_configuration[current_setup].duration) {
        set_next_setup(sim_configuration[current_setup], loc);

        time_acc = 0;
        repetition_counter++;

        // write error statistics
        struct particle p;
        p.x_pos = wb_gps_get_values(gps)[0];
        p.y_pos = wb_gps_get_values(gps)[1];

        struct particle estimate = loc.get_last_estimate();

        write_stats(error_stats_file, sim_configuration[current_setup], estimate, p);

        if(repetition_counter > current_setup_repetitions) {
          current_setup++;
          repetition_counter = 0;
          current_setup_repetitions = sim_configuration[current_setup].repetitions;
          current_setup_duration = sim_configuration[current_setup].duration;
        }
      }
    } else if (current_setup >= sizeof(sim_configuration)/sizeof(struct setup)) {
      break;
    }

    loc.tick();
  }

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
