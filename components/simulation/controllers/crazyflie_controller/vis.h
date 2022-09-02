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
  char name[80];
  char path[80];

  FILE * gnuplotPipe;

  enum plot_type type;

  // for plotting error
  int iteration;
};

void visualize_samples(struct vis_instance *vis, struct particle_filter_instance *pf_inst);
void visualize_error(struct vis_instance *vis, double error);
void create_vis(struct vis_instance *vis, const char *name, enum plot_type type);

#endif /* VIS_H */



