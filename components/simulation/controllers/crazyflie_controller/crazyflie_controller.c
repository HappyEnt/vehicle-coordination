/*
 * Copyright 2022 Bitcraze AB
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
 *  ...........       ____  _ __
 *  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 *  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 *  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 *
 * @file crazyflie_controller.c
 * Description: Controls the crazyflie in webots
 * Author:      Kimberly McGuire (Bitcraze AB)
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

// Visualization could be done using https://cyberbotics.com/doc/reference/display
// Should be exchanged later with a ros implementation See https://github.com/cyberbotics/webots_ros

#include <particle-belief-propagation.h>
#include <util.h>

// Add external controller
#include "pid_controller.h"
#include "vis.h"

#define PARTICLES 200


void clean_init_filter(struct particle_filter_instance **pf_inst) {

  if (pf_inst != NULL) {
    destroy_particle_filter_instance(*pf_inst);
  }

  create_particle_filter_instance(pf_inst);

  // create uniform distribution over space for now representing the prior knowledge.
  struct particle *own_particles = malloc(sizeof(struct particle) * PARTICLES);
  sample_from_2d_uniform(own_particles, PARTICLES, -40, 40, -40, 40);

  /* set_particle_array(*pf_inst, own_particles, PARTICLES); */
  set_particle_amount(*pf_inst, PARTICLES);

  set_filter_type(*pf_inst, POST_REGULARIZATION);

  pf_parallel_set_target_threads(8);

  set_receiver_std_dev(*pf_inst, 0.5);
}

void deinit_filter(struct particle_filter_instance *pf_inst) {
  if(pf_inst != NULL) {
    destroy_particle_filter_instance(pf_inst);
    pf_inst = NULL;
  }
}

int main(int argc, char **argv) {
  srand(time(0));

  struct particle_filter_instance *pf_inst = NULL;
  struct vis_instance vis;
  struct vis_instance vis_error;

  printf(" Initiliazing drone! \n");

  wb_robot_init();

  const int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize motors
  WbDeviceTag m1_motor = wb_robot_get_device("m1_motor");
  wb_motor_set_position(m1_motor, INFINITY);
  wb_motor_set_velocity(m1_motor, -1.0);
  WbDeviceTag m2_motor = wb_robot_get_device("m2_motor");
  wb_motor_set_position(m2_motor, INFINITY);
  wb_motor_set_velocity(m2_motor, 1.0);
  WbDeviceTag m3_motor = wb_robot_get_device("m3_motor");
  wb_motor_set_position(m3_motor, INFINITY);
  wb_motor_set_velocity(m3_motor, -1.0);
  WbDeviceTag m4_motor = wb_robot_get_device("m4_motor");
  wb_motor_set_position(m4_motor, INFINITY);
  wb_motor_set_velocity(m4_motor, 1.0);

  // Initialize sensors
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, timestep); // grep data from accelerometer every second
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag range_front = wb_robot_get_device("range_front");
  wb_distance_sensor_enable(range_front, timestep);
  WbDeviceTag range_left = wb_robot_get_device("range_left");
  wb_distance_sensor_enable(range_left, timestep);
  WbDeviceTag range_back = wb_robot_get_device("range_back");
  wb_distance_sensor_enable(range_back, timestep);
  WbDeviceTag range_right = wb_robot_get_device("range_right");
  wb_distance_sensor_enable(range_right, timestep);

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, 1);

  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, timestep);
  wb_receiver_set_channel(receiver, 1);


  // initialize particle filter
  clean_init_filter(&pf_inst);

  create_vis(&vis, wb_robot_get_name(), SCATTER_PLOT);

  char error_plot_name[80];
  sprintf(error_plot_name, "error-plot-%s",wb_robot_get_name());
  create_vis(&vis_error, error_plot_name, LINE_PLOT);

  // Initialize variables
  actual_state_t actual_state = {0};
  desired_state_t desired_state = {0};
  double past_x_global = 0;
  double past_y_global = 0;
  double past_time = wb_robot_get_time();
  double past_pf_iteration = wb_robot_get_time();
  double past_send_belief  = wb_robot_get_time();
  double past_acc_meas  = wb_robot_get_time();
  double dx_since_last = 0;
  double dy_since_last = 0;
  double vx = 0; // velocity in x. updated by acceloremeter measurements
  double vy = 0; // velocity in y.
  double constant_velocity_x = 0;
  double constant_velocity_y = 0;

  if(argc > 2) {
      constant_velocity_x = atof(argv[1]);
      constant_velocity_y = atof(argv[2]);
  }

  int has_fix = 0;
  double gps_last_x = wb_gps_get_values(gps)[0];
  double gps_last_y = wb_gps_get_values(gps)[1];

  // Initialize PID gains.
  gains_pid_t gains_pid;
  gains_pid.kp_att_y = 1;
  gains_pid.kd_att_y = 0.5;
  gains_pid.kp_att_rp = 0.5;
  gains_pid.kd_att_rp = 0.1;
  gains_pid.kp_vel_xy = 2;
  gains_pid.kd_vel_xy = 0.5;
  gains_pid.kp_z = 10;
  gains_pid.ki_z = 50;
  gains_pid.kd_z = 5;
  init_pid_attitude_fixed_height_controller();

  // Initialize struct for motor power
  motor_power_t motor_power;

  printf(" Take off! \n");
  printf("\n");

  printf("====== Controls =======\n");

  printf(" The Crazyflie can be controlled from your keyboard!\n");
  printf(" All controllable movement is in body coordinates\n");
  printf("- Use the up, back, right and left button to move in the horizontal plane\n");
  printf("- Use Q and E to rotate around yaw\n ");

  while (wb_robot_step(timestep) != -1) {
    const double dt = wb_robot_get_time() - past_time;
    const double dt_last_iteration = wb_robot_get_time() - past_pf_iteration;
    const double dt_last_send_belief = wb_robot_get_time() - past_send_belief;
    const double dt_last_acceleration_measurement = wb_robot_get_time() - past_acc_meas;

    // Get measurements
    actual_state.roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    actual_state.pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    actual_state.yaw_rate = wb_gyro_get_values(gyro)[2];

    // receiver measurements
    int queue_length = wb_receiver_get_queue_length(receiver);

    if (dt_last_iteration > 0.1 && wb_robot_get_time() > 5) {
      while(queue_length > 0) {
        const char *data = wb_receiver_get_data(receiver);
        int data_len = wb_receiver_get_data_size(receiver);
        int particles = data_len / sizeof(struct particle);
        double rssi = wb_receiver_get_signal_strength(receiver);

        // This can really happen ..... might be a bug in webots
        if(rssi <= 1e-9) {
          wb_receiver_next_packet(receiver);
          queue_length--;

          continue;
        }

        struct particle *foreign_particles = malloc(sizeof(struct particle) * particles);
        double measurement = sqrt((double)1.0 / rssi);

        memcpy(foreign_particles, data, sizeof(struct particle) * particles);

        struct message m = {
          .measured_distance = measurement,
          .particles = foreign_particles,
          .particles_length = particles,
          .type = DUMB_PARTICLES,
        };

        add_belief(pf_inst, m);

        free(foreign_particles);

        wb_receiver_next_packet(receiver);
        queue_length--;

        if(queue_length <= 0) {
          /* double dx = dx_since_last; */
          /* double dy = dy_since_last; */
          if(!has_fix) {
            gps_last_x = wb_gps_get_values(gps)[0];
            gps_last_y = wb_gps_get_values(gps)[1];
            has_fix = 1;
          }

          double gps_dx = (wb_gps_get_values(gps)[0] - gps_last_x);
          double gps_dy = (wb_gps_get_values(gps)[1] - gps_last_y);
          double gps_vx = gps_dx / dt_last_iteration;
          double gps_vy = gps_dy / dt_last_iteration;
          double dx = gps_dx;
          double dy = gps_dy;

          gps_last_x = wb_gps_get_values(gps)[0];
          gps_last_y = wb_gps_get_values(gps)[1];

          double dist = sqrt(dx*dx + dy*dy);

          /* printf("distance travelled. %f, %f\n", dx, dy); */
          /* printf("acc velocity. %f\n", vx); */
          /* printf("gps velocity. %f\n", gps_vx); */

          /* if(fabs(dist) > 1e-6) { */
          /*   printf("predict node %s", wb_robot_get_name()); */
          /*   predict_dist(pf_inst, dist); */
          /*   // instead of predicting next distance reset; */


          /*   /\* dx_since_last = 0; *\/ */
          /*   /\* dy_since_last = 0; *\/ */
          /* } */

          predict_max_movement_uniform(pf_inst, dt, 0.2); // assuming we have no real good way to measure the distance we moved, we should

          iterate(pf_inst);

          struct particle mean = estimate_position(pf_inst);

          past_pf_iteration = wb_robot_get_time();

          double error = sqrt((mean.x_pos - gps_last_x)*(mean.x_pos - gps_last_x) + (mean.y_pos - gps_last_y)*(mean.y_pos - gps_last_y));
          visualize_error(&vis_error, error);
          visualize_samples(&vis, pf_inst);
        }
      }
    }

    if(dt_last_send_belief > 0.5 && wb_robot_get_time() > 5) {
      struct particle *particles;
      size_t particle_length = get_particle_array(pf_inst, &particles);

      if(particles != NULL) {
        unsigned int bytes = sizeof(struct particle) * particle_length;
        char data[bytes];

        memcpy(data, particles, bytes);

        wb_emitter_send(emitter, data, bytes);
        past_send_belief = wb_robot_get_time();
      }
    }


    if (dt_last_acceleration_measurement * 1000 > wb_accelerometer_get_sampling_period(accelerometer)) {
      const double *acc_measurements = wb_accelerometer_get_values(accelerometer);
      const double step = (double) wb_accelerometer_get_sampling_period(accelerometer)/1000;
      vx += acc_measurements[0] * step;
      vy += acc_measurements[1] * step;
      dx_since_last += vx * step;
      dy_since_last += vy * step;

      past_acc_meas = wb_robot_get_time();
    }

    actual_state.altitude = wb_gps_get_values(gps)[2];
    double x_global = wb_gps_get_values(gps)[0];
    double vx_global = (x_global - past_x_global) / dt;
    double y_global = wb_gps_get_values(gps)[1];
    double vy_global = (y_global - past_y_global) / dt;

    // Get body fixed velocities
    double actualYaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    double cosyaw = cos(actualYaw);
    double sinyaw = sin(actualYaw);
    actual_state.vx = vx_global * cosyaw + vy_global * sinyaw;
    actual_state.vy = -vx_global * sinyaw + vy_global * cosyaw;

    // Initialize values
    desired_state.roll = 0;
    desired_state.pitch = 0;
    desired_state.vx = 0;
    desired_state.vy = 0;
    desired_state.yaw_rate = 0;
    desired_state.altitude = 1.0;

    double forward_desired = 0;
    double sideways_desired = 0;
    double yaw_desired = 0;

    // Control altitude
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          forward_desired = +0.2;
          break;
        case WB_KEYBOARD_DOWN:
          forward_desired = -0.2;
          break;
        case WB_KEYBOARD_RIGHT:
          sideways_desired = -0.2;
          break;
        case WB_KEYBOARD_LEFT:
          sideways_desired = +0.2;
          break;
        case 'Q':
          yaw_desired = 0.5;
          break;
        case 'E':
          yaw_desired = -0.5;
          break;
      }
      key = wb_keyboard_get_key();
    }

    // Example how to get sensor data
    // range_front_value = wb_distance_sensor_get_value(range_front));
    // const unsigned char *image = wb_camera_get_image(camera);

    desired_state.yaw_rate = yaw_desired;

    // PID velocity controller with fixed height
    desired_state.vy = sideways_desired + constant_velocity_y;
    desired_state.vx = forward_desired + constant_velocity_x;
    pid_velocity_fixed_height_controller(actual_state, &desired_state, gains_pid, dt, &motor_power);

    // Setting motorspeed
    wb_motor_set_velocity(m1_motor, -motor_power.m1);
    wb_motor_set_velocity(m2_motor, motor_power.m2);
    wb_motor_set_velocity(m3_motor, -motor_power.m3);
    wb_motor_set_velocity(m4_motor, motor_power.m4);

    // Save past time for next time step
    past_time = wb_robot_get_time();
    past_x_global = x_global;
    past_y_global = y_global;
  };

  wb_robot_cleanup();

  return 0;
}
