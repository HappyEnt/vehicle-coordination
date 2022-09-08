/*
 * File:          anchor_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/gps.h>

#include <util.h>

#/*
 * You may want to add macros here.
 */

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();


  const int timestep = (int)wb_robot_get_basic_time_step();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, 1);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);

  wb_emitter_set_channel(emitter, 1);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  double past_time = wb_robot_get_time();

  while (wb_robot_step(timestep) != -1) {
    const double dt = wb_robot_get_time() - past_time;

    // send position every 100 milliseconds
    if(dt > 0.025 && wb_robot_get_time() > 5) {
      char data[sizeof(struct particle)];

      double x_global = wb_gps_get_values(gps)[0];
      double y_global = wb_gps_get_values(gps)[1];

      struct particle p = {
        .x_pos = x_global,
        .y_pos = y_global,
        /* .orientation = 0.0, */
        .weight = 1.0
      };

      memcpy(data, &p, sizeof(struct particle));

      wb_emitter_send(emitter, data, sizeof(struct particle));
      past_time = wb_robot_get_time();
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
