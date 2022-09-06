#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <util.h>
#include <time.h>
#include <unistd.h>

#include "vis.h"

#include <particle-belief-propagation.h>

void write_particles(struct vis_instance *vis, struct particle_filter_instance *pf_inst) {
  struct particle *particles;
  size_t amount = get_particle_array(pf_inst, &particles);

  FILE *data = fopen(vis->particles_path, "w");

  for (unsigned int i = 0; i < amount; ++i) {
    double x = particles[i].x_pos;
    double y = particles[i].y_pos;
    fprintf(data, "%f %f \n", x, y); //Write the data to a temporary file
  }

  fflush(data);

  fclose(data);
}

void write_error(struct vis_instance *vis, double error) {
  FILE *data = fopen(vis->error_path, "a");

  vis->iteration++;
  fprintf(data, "%d %f \n", vis->iteration, error); //Write the data to a temporary file

  fflush(data);

  fclose(data);
}

void create_vis(struct vis_instance *vis, const char *name) {
  vis->iteration = 0;

  strcpy(vis->node_name, name);
  sprintf(vis->error_path, "%s-error.data", vis->node_name);
  sprintf(vis->particles_path, "%s-particles.data", vis->node_name);

  char gnuplot_cmd[200];
  sprintf(gnuplot_cmd, "gnuplot -noraise -persist -e \"nodename='%s'\" node-plot.plg", vis->node_name);

  vis->gnuplotPipe = popen (gnuplot_cmd, "w");

  FILE *error_data = fopen(vis->error_path,"w");
  FILE *particles_data = fopen(vis->particles_path,"w");

  fprintf(error_data, "%d %f \n", 0, 0.0); //Write the data to a temporary file
  fprintf(particles_data, "%f %f \n", 0.0, 0.0); //Write the data to a temporary file
  /* fflush(data); */

  fclose(error_data);
  fclose(particles_data);

  fflush(vis->gnuplotPipe);
}
