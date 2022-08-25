#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <util.h>
#include <time.h>
#include <unistd.h>

#include "vis.h"

#include <particle-belief-propagation.h>

#define GNUPLOT "gnuplot -persist"
/* #define GNUPLOT "cat"	// to test output */

/* void vis(struct particle_filter_instance *pf_inst) */
void visualize(struct vis_instance *vis, struct particle_filter_instance *pf_inst)
{
  struct particle *particles = pf_inst->local_particles;
  size_t amount = pf_inst->local_particles_length;

  if (ftruncate(fileno(vis->data), 0) == -1) {
    printf("Could not truncate\n");
  }
  fflush(vis->data);

  for (unsigned int i = 0; i < amount; ++i) {
    double x = particles[i].x_pos;
    double y = particles[i].y_pos;
    fprintf(vis->data, "%lf %lf \n", x, y); //Write the data to a temporary file
  }

  fflush(vis->data);

  char * commandsForGnuplot[] = {"replot"};
  int commands = sizeof(commandsForGnuplot) / sizeof(char*);

  for (size_t i=0; i < commands; i++)
    {
      fprintf(vis->gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.
    }

  fflush(vis->gnuplotPipe);
}

void create_vis(struct vis_instance *vis, const char *name) {
  char * commandsForGnuplot[] = {"set title \"TITLEEEEE\"", "set xrange [-5:5]", "set yrange [-5:5]", "set term qt noraise"};
  int commands = sizeof(commandsForGnuplot) / sizeof(char*);

  char path[80];
  char plot_cmd[80];

  strcpy(vis->name, name);

  sprintf(path, "data-%s.data", vis->name);
  sprintf(plot_cmd, "plot '%s' with points pt 3", path);

  vis->gnuplotPipe = popen ("gnuplot -persistent", "w");

  vis->data = fopen(path,"w");

  fprintf(vis->data, "%lf %lf \n", 0.0, 0.0); //Write the data to a temporary file
  fflush(vis->data);

  printf("sending %d commands \n", commands);


  for (size_t i=0; i < commands; i++)
    {
      fprintf(vis->gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.
    }

  fprintf(vis->gnuplotPipe, "%s \n", plot_cmd); //Send commands to gnuplot one by one.

  fflush(vis->gnuplotPipe);
}
