#ifndef VIS_H
#define VIS_H

#include <particle-belief-propagation.h>
#include <stdio.h>
#include <stdlib.h>


enum plot_type {
  LINE_PLOT,
  SCATTER_PLOT
};


struct vis_instance {
  char node_name[80];
  char error_path[80];
  char particles_path[80];

  FILE * gnuplotPipe;

  // for plotting error
  int iteration;
};

void write_error(struct vis_instance *vis, double error);
void write_particles(struct vis_instance *vis, struct particle_filter_instance *pf_inst);

void create_vis(struct vis_instance *vis, const char *name);

#endif /* VIS_H */



